#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "standalone_global_planner.h"

// Standalone pipeline:
// /map -> inflated binary grid -> A* search -> /plan
// /slam_pose is treated as the start pose. The node expects pose, goal, and map
// to already be in the same frame because this first version does not use TF.
StandaloneGlobalPlanner::StandaloneGlobalPlanner(): Node("standalone_global_planner"){
  // Topic and behavior parameters keep the node reusable without recompiling.
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/slam_pose");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
  obstacle_threshold_ = declare_parameter<int>("obstacle_threshold", 50);
  allow_unknown_ = declare_parameter<bool>("allow_unknown", true);
  robot_radius_ = declare_parameter<double>("robot_radius", 0.25);
  use_diagonal_ = declare_parameter<bool>("use_diagonal", true);
  max_snap_distance_ = declare_parameter<double>("max_snap_distance", 0.5);
  map_transient_local_ = declare_parameter<bool>("map_transient_local", false);

  obstacle_threshold_ = std::clamp(obstacle_threshold_, 0, 100);
  robot_radius_ = std::max(0.0, robot_radius_);
  max_snap_distance_ = std::max(0.0, max_snap_distance_);

  // Static maps are sometimes published with transient_local durability.
  // Keep the default volatile mode unless the parameter is enabled.
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  if (map_transient_local_) {
    map_qos.transient_local();
  }

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, map_qos,
    std::bind(&StandaloneGlobalPlanner::mapCallback, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, rclcpp::QoS(10),
    std::bind(&StandaloneGlobalPlanner::poseCallback, this, std::placeholders::_1));

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&StandaloneGlobalPlanner::goalCallback, this, std::placeholders::_1));

  plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic_, rclcpp::QoS(10));

  RCLCPP_INFO(
    get_logger(),
    "Standalone planner ready: map=%s pose=%s goal=%s plan=%s radius=%.3f m",
    map_topic_.c_str(), pose_topic_.c_str(), goal_topic_.c_str(), plan_topic_.c_str(),
    robot_radius_);
}

void StandaloneGlobalPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  if (msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
    RCLCPP_WARN(get_logger(), "Ignoring invalid map message.");
    return;
  }

  // Rebuild inflation whenever the map changes so planning uses the latest
  // obstacle clearance.
  map_ = msg;
  buildInflatedGrid();
  has_map_ = true;

  RCLCPP_INFO_ONCE(
    get_logger(), "Received map: %u x %u, resolution %.3f m/cell",
    map_->info.width, map_->info.height, map_->info.resolution);

  if (has_pose_ && has_goal_) {
    makeAndPublishPlan();
  }
}

void StandaloneGlobalPlanner::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  // Covariance is ignored for planning. Only the estimated pose is used as
  // the start pose.
  current_pose_.header = msg->header;
  current_pose_.pose = msg->pose.pose;
  has_pose_ = true;
}

void StandaloneGlobalPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  // A new goal immediately triggers planning if map and pose are ready.
  goal_pose_ = *msg;
  has_goal_ = true;
  makeAndPublishPlan();
}

void StandaloneGlobalPlanner::makeAndPublishPlan(){
  // The planner needs all three inputs before it can create a path.
  if (!has_map_) {
    RCLCPP_WARN(get_logger(), "Cannot plan yet: no map received.");
    return;
  }
  if (!has_pose_) {
    RCLCPP_WARN(get_logger(), "Cannot plan yet: no /slam_pose received.");
    return;
  }

  warnIfFrameMismatch(current_pose_.header.frame_id, "map");
  warnIfFrameMismatch(goal_pose_.header.frame_id, "goal");

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!worldToMap(current_pose_.pose.position, start_x, start_y)) {
    RCLCPP_WARN(get_logger(), "Start pose is outside the map.");
    return;
  }
  if (!worldToMap(goal_pose_.pose.position, goal_x, goal_y)) {
    RCLCPP_WARN(get_logger(), "Goal pose is outside the map.");
    return;
  }

  int start_index = toIndex(start_x, start_y);
  int goal_index = toIndex(goal_x, goal_y);

  // If start or goal lands inside inflated space, try a small local snap to
  // the nearest free cell. This makes RViz goal clicks near walls less brittle.
  const int snap_radius_cells =
    static_cast<int>(std::ceil(max_snap_distance_ / map_->info.resolution));

  const int original_start = start_index;
  const int original_goal = goal_index;
  if (!findNearestFreeCell(start_index, snap_radius_cells, start_index)) {
    RCLCPP_WARN(get_logger(), "Start pose is inside an obstacle or inflated area.");
    return;
  }
  if (!findNearestFreeCell(goal_index, snap_radius_cells, goal_index)) {
    RCLCPP_WARN(get_logger(), "Goal pose is inside an obstacle or inflated area.");
    return;
  }
  if (start_index != original_start) {
    RCLCPP_WARN(get_logger(), "Start pose snapped to the nearest free cell.");
  }
  if (goal_index != original_goal) {
    RCLCPP_WARN(get_logger(), "Goal pose snapped to the nearest free cell.");
  }

  const auto path_indices = runAStar(start_index, goal_index);
  if (path_indices.empty()) {
    RCLCPP_WARN(get_logger(), "No path found from start to goal.");
    return;
  }

  // Convert grid indices back into a nav_msgs/Path in the map frame.
  auto path = buildPathMessage(path_indices);
  plan_pub_->publish(path);

  RCLCPP_INFO(get_logger(), "Published global path with %zu poses.", path.poses.size());
}

void StandaloneGlobalPlanner::buildInflatedGrid() {
  // inflated_grid_ is a binary planning grid:
  //   0 = traversable
  //   1 = occupied or inside robot-radius inflation
  const auto width = static_cast<int>(map_->info.width);
  const auto height = static_cast<int>(map_->info.height);
  inflated_grid_.assign(static_cast<size_t>(width * height), 0);

  // First collect obstacle cells from OccupancyGrid values.
  std::vector<int> obstacle_indices;
  obstacle_indices.reserve(inflated_grid_.size() / 8);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = toIndex(x, y);
      if (isObstacleValue(map_->data[static_cast<size_t>(index)])) {
        obstacle_indices.push_back(index);
      }
    }
  }

  const double radius_cells = robot_radius_ / map_->info.resolution;
  const int radius_limit = static_cast<int>(std::ceil(radius_cells));
  const double radius_squared = radius_cells * radius_cells;

  // Mark every cell within robot_radius of each obstacle as blocked.
  for (const int obstacle_index : obstacle_indices) {
    const int obstacle_x = obstacle_index % width;
    const int obstacle_y = obstacle_index / width;

    for (int dy = -radius_limit; dy <= radius_limit; ++dy) {
      for (int dx = -radius_limit; dx <= radius_limit; ++dx) {
        if (static_cast<double>(dx * dx + dy * dy) > radius_squared) {
          continue;
        }

        const int nx = obstacle_x + dx;
        const int ny = obstacle_y + dy;
        if (!inBounds(nx, ny)) {
          continue;
        }

        inflated_grid_[static_cast<size_t>(toIndex(nx, ny))] = 1;
      }
    }
  }
}

std::vector<int> StandaloneGlobalPlanner::runAStar(const int start_index, const int goal_index) const {
  // Standard A*: g_score stores cost from the start, f_score = g + heuristic.
  const int total_cells = static_cast<int>(inflated_grid_.size());
  const double infinity = std::numeric_limits<double>::infinity();

  std::vector<double> g_score(static_cast<size_t>(total_cells), infinity);
  std::vector<int> came_from(static_cast<size_t>(total_cells), -1);
  std::vector<uint8_t> closed(static_cast<size_t>(total_cells), 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open_set;

  g_score[static_cast<size_t>(start_index)] = 0.0;
  open_set.push({start_index, heuristic(start_index, goal_index)});

  while (!open_set.empty()) {
    const int current = open_set.top().index;
    open_set.pop();

    // A node can be pushed more than once with better scores. Closed nodes
    // let us skip stale queue entries.
    if (closed[static_cast<size_t>(current)] != 0) {
      continue;
    }
    if (current == goal_index) {
      return reconstructPath(came_from, start_index, goal_index);
    }

    closed[static_cast<size_t>(current)] = 1;

    const int current_x = current % static_cast<int>(map_->info.width);
    const int current_y = current / static_cast<int>(map_->info.width);

    for (const auto & neighbor : neighbors(current_x, current_y)) {
      const int neighbor_x = neighbor.first;
      const int neighbor_y = neighbor.second;
      const int neighbor_index = toIndex(neighbor_x, neighbor_y);

      if (closed[static_cast<size_t>(neighbor_index)] != 0) {
        continue;
      }

      const double step_cost =
        std::hypot(neighbor_x - current_x, neighbor_y - current_y) *
        traversalCost(neighbor_index);
      const double tentative_g = g_score[static_cast<size_t>(current)] + step_cost;

      // Relax the edge if this route gives a cheaper path to the neighbor.
      if (tentative_g < g_score[static_cast<size_t>(neighbor_index)]) {
        came_from[static_cast<size_t>(neighbor_index)] = current;
        g_score[static_cast<size_t>(neighbor_index)] = tentative_g;
        open_set.push(
          {neighbor_index, tentative_g + heuristic(neighbor_index, goal_index)});
      }
    }
  }

  return {};
}

std::vector<std::pair<int, int>> StandaloneGlobalPlanner::neighbors(const int x, const int y) const {
  // The first four directions are cardinal moves. The last four add diagonals
  // when use_diagonal is enabled.
  static constexpr int dx8[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  static constexpr int dy8[8] = {0, 1, 0, -1, 1, -1, 1, -1};
  const int count = use_diagonal_ ? 8 : 4;

  std::vector<std::pair<int, int>> result;
  result.reserve(static_cast<size_t>(count));

  for (int i = 0; i < count; ++i) {
    const int nx = x + dx8[i];
    const int ny = y + dy8[i];
    if (!isFree(nx, ny)) {
      continue;
    }

    if (dx8[i] != 0 && dy8[i] != 0) {
      // Prevent diagonal corner-cutting through two touching obstacles.
      if (!isFree(nx, y) || !isFree(x, ny)) {
        continue;
      }
    }

    result.emplace_back(nx, ny);
  }

  return result;
}

std::vector<int> StandaloneGlobalPlanner::reconstructPath(
  const std::vector<int> & came_from,
  const int start_index,
  const int goal_index) const {
  std::vector<int> path;
  int current = goal_index;

  while (current != -1) {
    path.push_back(current);
    if (current == start_index) {
      break;
    }
    current = came_from[static_cast<size_t>(current)];
  }

  if (path.empty() || path.back() != start_index) {
    return {};
  }

  std::reverse(path.begin(), path.end());
  return path;
}

nav_msgs::msg::Path StandaloneGlobalPlanner::buildPathMessage(const std::vector<int> & path_indices) const {
  // Path poses are placed at cell centers. Orientation faces the next pose;
  // the final pose keeps the requested goal orientation.
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = mapFrame();
  path.poses.reserve(path_indices.size());

  for (const int index : path_indices) {
    const int x = index % static_cast<int>(map_->info.width);
    const int y = index / static_cast<int>(map_->info.width);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = mapToWorld(x, y);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  for (size_t i = 0; i < path.poses.size(); ++i) {
    if (i + 1 == path.poses.size()) {
      path.poses[i].pose.orientation = goal_pose_.pose.orientation;
      continue;
    }

    const auto & current = path.poses[i].pose.position;
    const auto & next = path.poses[i + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    path.poses[i].pose.orientation = yawToQuaternion(yaw);
  }

  return path;
}

bool StandaloneGlobalPlanner::findNearestFreeCell(
  const int source_index,
  const int max_radius_cells,
  int & free_index) const {
  // Used for start/goal recovery when a pose lies just inside inflated space.
  if (isFree(source_index)) {
    free_index = source_index;
    return true;
  }

  const int width = static_cast<int>(map_->info.width);
  const int source_x = source_index % width;
  const int source_y = source_index / width;

  for (int radius = 1; radius <= max_radius_cells; ++radius) {
    int best_index = -1;
    int best_distance_squared = std::numeric_limits<int>::max();

    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != radius) {
          continue;
        }
        if (dx * dx + dy * dy > max_radius_cells * max_radius_cells) {
          continue;
        }

        const int nx = source_x + dx;
        const int ny = source_y + dy;
        if (!isFree(nx, ny)) {
          continue;
        }

        const int distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_distance_squared) {
          best_distance_squared = distance_squared;
          best_index = toIndex(nx, ny);
        }
      }
    }

    if (best_index >= 0) {
      free_index = best_index;
      return true;
    }
  }

  return false;
}

bool StandaloneGlobalPlanner::worldToMap(const geometry_msgs::msg::Point & point, int & mx, int & my) const {
  // Convert world coordinates into map-grid coordinates. OccupancyGrid origin
  // can include yaw, so rotate into the map's local grid frame first.
  const auto & origin = map_->info.origin.position;
  const double yaw = yawFromQuaternion(map_->info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double dx = point.x - origin.x;
  const double dy = point.y - origin.y;
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;

  if (local_x < 0.0 || local_y < 0.0) {
    return false;
  }

  mx = static_cast<int>(std::floor(local_x / map_->info.resolution));
  my = static_cast<int>(std::floor(local_y / map_->info.resolution));
  return inBounds(mx, my);
}

geometry_msgs::msg::Point StandaloneGlobalPlanner::mapToWorld(const int mx, const int my) const {
  // Convert a grid cell back to the world coordinate at the cell center.
  const auto & origin = map_->info.origin.position;
  const double yaw = yawFromQuaternion(map_->info.origin.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double local_x = (static_cast<double>(mx) + 0.5) * map_->info.resolution;
  const double local_y = (static_cast<double>(my) + 0.5) * map_->info.resolution;

  geometry_msgs::msg::Point point;
  point.x = origin.x + cos_yaw * local_x - sin_yaw * local_y;
  point.y = origin.y + sin_yaw * local_x + cos_yaw * local_y;
  point.z = 0.0;
  return point;
}

double StandaloneGlobalPlanner::heuristic(const int from_index, const int to_index) const {
  const int width = static_cast<int>(map_->info.width);
  const int from_x = from_index % width;
  const int from_y = from_index / width;
  const int to_x = to_index % width;
  const int to_y = to_index / width;

  // calculate the Euclidean distance between two grid cells as the heuristic
  return std::hypot(to_x - from_x, to_y - from_y);
}

double StandaloneGlobalPlanner::traversalCost(const int index) const {
  // Free cells cost 1. Non-obstacle occupancy values add a small preference
  // for lower-cost cells while still allowing traversal.
  const int8_t value = map_->data[static_cast<size_t>(index)];
  if (value <= 0) {
    return 1.0;
  }

  return 1.0 + static_cast<double>(value) / 100.0;
}

bool StandaloneGlobalPlanner::isObstacleValue(const int8_t value) const {
  // Unknown cells (-1) can be either blocked or allowed by parameter.
  if (value < 0) {
    return !allow_unknown_;
  }

  return value >= obstacle_threshold_;
}

bool StandaloneGlobalPlanner::isFree(const int index) const {
  if (index < 0 || index >= static_cast<int>(inflated_grid_.size())) {
    return false;
  }

  return inflated_grid_[static_cast<size_t>(index)] == 0;
}

bool StandaloneGlobalPlanner::inBounds(const int x, const int y) const {
  return x >= 0 && y >= 0 &&
          x < static_cast<int>(map_->info.width) &&
          y < static_cast<int>(map_->info.height);
}

void StandaloneGlobalPlanner::warnIfFrameMismatch(const std::string & frame_id, const char * source_name) const {
  if (frame_id.empty() || frame_id == mapFrame()) {
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "%s frame_id is '%s' but map frame_id is '%s'. This standalone planner does not TF-transform poses.",
    source_name, frame_id.c_str(), mapFrame().c_str());
}

double StandaloneGlobalPlanner::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion StandaloneGlobalPlanner::yawToQuaternion(const double yaw) const{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StandaloneGlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
