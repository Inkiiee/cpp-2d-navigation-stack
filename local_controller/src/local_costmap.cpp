#include "local_costmap.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"

LocalCostmap::LocalCostmap(
  double width, double height, double resolution,
  double robot_radius, double safety_margin, double obstacle_max_range)
: width_(std::max(width, 0.1)),
  height_(std::max(height, 0.1)),
  resolution_(std::max(resolution, 0.01)),
  inflation_radius_(std::max(robot_radius + safety_margin, 0.0)),
  obstacle_max_range_(std::max(obstacle_max_range, 0.1)),
  cells_x_(static_cast<int>(std::ceil(width_ / resolution_))),
  cells_y_(static_cast<int>(std::ceil(height_ / resolution_)))
{
  // 요청 크기를 셀 수에 맞춰 올림했으므로 실제 meter 크기를 다시 계산한다.
  width_ = static_cast<double>(cells_x_) * resolution_;
  height_ = static_cast<double>(cells_y_) * resolution_;
  grid_.assign(static_cast<size_t>(cells_x_ * cells_y_), 0);
  clearance_grid_.assign(
    static_cast<size_t>(cells_x_ * cells_y_), std::max(width_, height_));
}

bool LocalCostmap::update(const sensor_msgs::msg::PointCloud2 & cloud)
{
  // 매 point cloud마다 과거 격자를 지우는 로봇 중심 rolling snapshot 방식이다.
  // 따라서 움직이는 장애물이 이전 위치에 계속 남는 현상을 방지한다.
  std::fill(grid_.begin(), grid_.end(), 0);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      // 너무 먼 벽까지 회피 대상으로 잡으면 가까운 주행 공간이 과도하게
      // 좁아질 수 있으므로 로봇 중심 반경으로 사용 범위를 제한한다.
      if (std::hypot(x, y) > obstacle_max_range_) {
        continue;
      }

      int gx = 0;
      int gy = 0;
      // Local Costmap 범위 안에 들어온 endpoint만 로봇 크기만큼 팽창한다.
      if (worldToGrid(x, y, gx, gy)) {
        markInflatedObstacle(x, y);
      }
    }
  } catch (const std::runtime_error &) {
    std::fill(
      clearance_grid_.begin(), clearance_grid_.end(),
      std::max(width_, height_));
    return false;
  }

  buildClearanceField();
  return true;
}

bool LocalCostmap::isCollision(double robot_x, double robot_y) const
{
  int gx = 0;
  int gy = 0;
  // 예측 궤적이 Local Costmap 바깥으로 나가면 관측 불가능 영역이므로
  // 안전을 위해 충돌로 취급한다.
  if (!worldToGrid(robot_x, robot_y, gx, gy)) {
    return true;
  }
  return grid_[static_cast<size_t>(index(gx, gy))] >= 100;
}

double LocalCostmap::obstacleClearance(double robot_x, double robot_y) const
{
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(robot_x, robot_y, gx, gy)) {
    return 0.0;
  }
  return clearance_grid_[static_cast<size_t>(index(gx, gy))];
}

nav_msgs::msg::OccupancyGrid LocalCostmap::toMessage(
  const rclcpp::Time & stamp, const std::string & frame_id) const
{
  nav_msgs::msg::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.resolution = static_cast<float>(resolution_);
  message.info.width = static_cast<uint32_t>(cells_x_);
  message.info.height = static_cast<uint32_t>(cells_y_);

  // 격자 중심이 항상 로봇 원점 (0, 0)이 되도록 좌측 아래 origin을
  // (-width/2, -height/2)에 둔다.
  message.info.origin.position.x = -width_ * 0.5;
  message.info.origin.position.y = -height_ * 0.5;
  message.info.origin.orientation.w = 1.0;
  message.data = grid_;
  return message;
}

bool LocalCostmap::worldToGrid(double x, double y, int & gx, int & gy) const
{
  // 로봇은 격자 중앙에 있으므로 전체 길이의 절반을 더해 양수 셀로 바꾼다.
  gx = static_cast<int>(std::floor((x + width_ * 0.5) / resolution_));
  gy = static_cast<int>(std::floor((y + height_ * 0.5) / resolution_));
  return gx >= 0 && gy >= 0 && gx < cells_x_ && gy < cells_y_;
}

int LocalCostmap::index(int gx, int gy) const
{
  return gy * cells_x_ + gx;
}

void LocalCostmap::markInflatedObstacle(double x, double y)
{
  int center_x = 0;
  int center_y = 0;
  if (!worldToGrid(x, y, center_x, center_y)) {
    return;
  }

  const int radius_cells =
    static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const double radius_squared = inflation_radius_ * inflation_radius_;

  // endpoint를 중심으로 원형 영역만 100으로 표시한다.
  // 이후에는 로봇을 점으로 보고 궤적 중심만 검사해도 footprint 충돌을
  // 판단할 수 있다.
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int gx = center_x + dx;
      const int gy = center_y + dy;
      if (gx < 0 || gy < 0 || gx >= cells_x_ || gy >= cells_y_) {
        continue;
      }

      const double distance_squared =
        static_cast<double>(dx * dx + dy * dy) * resolution_ * resolution_;
      if (distance_squared <= radius_squared) {
        grid_[static_cast<size_t>(index(gx, gy))] = 100;
      }
    }
  }
}

void LocalCostmap::buildClearanceField()
{
  // inflation cell들을 시작점으로 하는 multi-source Dijkstra이다.
  // Costmap은 기본 80x80 정도이므로 scan마다 계산해도 후보 trajectory에서
  // 모든 원본 scan point를 반복 순회하는 것보다 훨씬 저렴하다.
  const double infinity = std::numeric_limits<double>::infinity();
  clearance_grid_.assign(grid_.size(), infinity);

  using QueueEntry = std::pair<double, int>;
  std::priority_queue<
    QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  for (int cell = 0; cell < static_cast<int>(grid_.size()); ++cell) {
    if (grid_[static_cast<size_t>(cell)] >= 100) {
      clearance_grid_[static_cast<size_t>(cell)] = 0.0;
      open.emplace(0.0, cell);
    }
  }

  if (open.empty()) {
    std::fill(
      clearance_grid_.begin(), clearance_grid_.end(),
      std::max(width_, height_));
    return;
  }

  static constexpr int dx[8] = {1, 0, -1, 0, 1, 1, -1, -1};
  static constexpr int dy[8] = {0, 1, 0, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const auto [distance, cell] = open.top();
    open.pop();
    if (distance > clearance_grid_[static_cast<size_t>(cell)]) {
      continue;
    }

    const int x = cell % cells_x_;
    const int y = cell / cells_x_;
    for (int i = 0; i < 8; ++i) {
      const int nx = x + dx[i];
      const int ny = y + dy[i];
      if (nx < 0 || ny < 0 || nx >= cells_x_ || ny >= cells_y_) {
        continue;
      }

      const int next = index(nx, ny);
      const double step =
        (dx[i] == 0 || dy[i] == 0) ? resolution_ :
        resolution_ * std::sqrt(2.0);
      const double candidate = distance + step;
      if (candidate < clearance_grid_[static_cast<size_t>(next)]) {
        clearance_grid_[static_cast<size_t>(next)] = candidate;
        open.emplace(candidate, next);
      }
    }
  }
}
