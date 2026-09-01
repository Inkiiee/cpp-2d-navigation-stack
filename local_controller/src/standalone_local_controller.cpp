#include "standalone_local_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

StandaloneLocalController::StandaloneLocalController()
: Node("standalone_local_controller"),
  last_pose_received_(0, 0, RCL_ROS_TIME),
  last_scan_received_(0, 0, RCL_ROS_TIME),
  state_since_(0, 0, RCL_ROS_TIME)
{
  // 토픽 이름과 robot frame은 파라미터로 열어 두어 remap 없이도 변경할 수 있다.
  plan_topic_ = declare_parameter<std::string>("plan_topic", "/plan");
  pose_topic_ = declare_parameter<std::string>("pose_topic", "/slam_pose");
  goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
  scan_topic_ =
    declare_parameter<std::string>("scan_topic", "/scan_deskewed");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  costmap_topic_ =
    declare_parameter<std::string>("costmap_topic", "/local_costmap");
  robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

  // Pure Pursuit와 목표 도착 판정 파라미터.
  control_frequency_ = declare_parameter<double>("control_frequency", 20.0);
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 0.4);
  max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.2);
  max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.8);
  goal_position_tolerance_ =
    declare_parameter<double>("goal_position_tolerance", 0.1);
  goal_yaw_tolerance_ = declare_parameter<double>("goal_yaw_tolerance", 0.15);
  plan_goal_match_tolerance_ =
    declare_parameter<double>("plan_goal_match_tolerance", 0.75);

  // 입력 timeout, 미래 궤적, 상태 유지 파라미터.
  pose_timeout_ = declare_parameter<double>("pose_timeout", 1.5);
  scan_timeout_ = declare_parameter<double>("scan_timeout", 0.5);
  prediction_horizon_ = declare_parameter<double>("prediction_horizon", 1.5);
  prediction_step_ = declare_parameter<double>("prediction_step", 0.1);
  avoidance_hold_time_ =
    declare_parameter<double>("avoidance_hold_time", 0.8);
  blocked_rotate_speed_ =
    declare_parameter<double>("blocked_rotate_speed", 0.3);
  safe_cycles_required_ =
    declare_parameter<int>("safe_cycles_required", 5);

  // 속도 변화 제한과 DWA-like sampling 개수.
  max_linear_acceleration_ =
    declare_parameter<double>("max_linear_acceleration", 0.3);
  max_angular_acceleration_ =
    declare_parameter<double>("max_angular_acceleration", 1.5);
  linear_samples_ = declare_parameter<int>("linear_samples", 5);
  angular_samples_ = declare_parameter<int>("angular_samples", 21);

  // 후보 trajectory 점수 가중치.
  path_weight_ = declare_parameter<double>("path_weight", 2.0);
  target_weight_ = declare_parameter<double>("target_weight", 1.0);
  heading_weight_ = declare_parameter<double>("heading_weight", 1.0);
  obstacle_weight_ = declare_parameter<double>("obstacle_weight", 0.4);
  velocity_weight_ = declare_parameter<double>("velocity_weight", 0.5);

  const double costmap_width =
    declare_parameter<double>("costmap_width", 4.0);
  const double costmap_height =
    declare_parameter<double>("costmap_height", 4.0);
  const double costmap_resolution =
    declare_parameter<double>("costmap_resolution", 0.05);
  const double robot_radius =
    declare_parameter<double>("robot_radius", 0.25);
  const double safety_margin =
    declare_parameter<double>("safety_margin", 0.05);
  const double obstacle_max_range =
    declare_parameter<double>("obstacle_max_range", 2.0);

  // 0 또는 음수 파라미터 때문에 나눗셈/무한 루프가 생기지 않도록 제한한다.
  control_frequency_ = std::max(control_frequency_, 1.0);
  lookahead_distance_ = std::max(lookahead_distance_, 0.01);
  max_linear_velocity_ = std::max(max_linear_velocity_, 0.0);
  max_angular_velocity_ = std::max(max_angular_velocity_, 0.0);
  goal_position_tolerance_ = std::max(goal_position_tolerance_, 0.0);
  goal_yaw_tolerance_ = std::max(goal_yaw_tolerance_, 0.0);
  plan_goal_match_tolerance_ = std::max(plan_goal_match_tolerance_, 0.0);
  pose_timeout_ = std::max(pose_timeout_, 0.0);
  scan_timeout_ = std::max(scan_timeout_, 0.0);
  prediction_horizon_ = std::max(prediction_horizon_, 0.1);
  prediction_step_ = std::clamp(prediction_step_, 0.01, prediction_horizon_);
  avoidance_hold_time_ = std::max(avoidance_hold_time_, 0.0);
  blocked_rotate_speed_ =
    std::clamp(blocked_rotate_speed_, 0.0, max_angular_velocity_);
  max_linear_acceleration_ = std::max(max_linear_acceleration_, 0.01);
  max_angular_acceleration_ = std::max(max_angular_acceleration_, 0.01);
  linear_samples_ = std::max(linear_samples_, 2);
  angular_samples_ = std::max(angular_samples_, 3);
  safe_cycles_required_ = std::max(safe_cycles_required_, 1);
  path_weight_ = std::max(path_weight_, 0.0);
  target_weight_ = std::max(target_weight_, 0.0);
  heading_weight_ = std::max(heading_weight_, 0.0);
  obstacle_weight_ = std::max(obstacle_weight_, 0.0);
  velocity_weight_ = std::max(velocity_weight_, 0.0);

  local_costmap_ = std::make_unique<LocalCostmap>(
    costmap_width, costmap_height, costmap_resolution,
    robot_radius, safety_margin, obstacle_max_range);

  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    plan_topic_, rclcpp::QoS(10),
    std::bind(
      &StandaloneLocalController::planCallback, this,
      std::placeholders::_1));

  // map이 아직 없어 /plan이 나오지 않아도 직접 목표를 추종하기 위해 구독한다.
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(
      &StandaloneLocalController::goalCallback, this,
      std::placeholders::_1));

  pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic_, rclcpp::QoS(10),
    std::bind(
      &StandaloneLocalController::poseCallback, this,
      std::placeholders::_1));

  // test_icp에서 scan 종료 시각의 base_link 기준으로 deskew한 점군이다.
  scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(
      &StandaloneLocalController::scanCallback, this,
      std::placeholders::_1));

  cmd_vel_pub_ =
    create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(1));

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_frequency_));
  control_timer_ = create_wall_timer(
    timer_period,
    std::bind(&StandaloneLocalController::controlLoop, this));
  state_since_ = now();

  RCLCPP_INFO(
    get_logger(),
    "Local controller ready: plan=%s goal=%s pose=%s cloud=%s cmd_vel=%s rate=%.1f Hz",
    plan_topic_.c_str(), goal_topic_.c_str(), pose_topic_.c_str(),
    scan_topic_.c_str(), cmd_vel_topic_.c_str(), control_frequency_);
}

void StandaloneLocalController::planCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    has_plan_ = false;
    current_plan_.poses.clear();
    changeState(nominalState());
    RCLCPP_WARN(
      get_logger(), "Received an empty path; using goal-seeking fallback.");
    return;
  }

  // 새 goal 직후 과거 goal의 늦은 plan이 도착하면 사용하지 않는다.
  if (has_goal_ && !planMatchesCurrentGoal(*msg)) {
    RCLCPP_WARN(
      get_logger(),
      "Ignoring a path whose endpoint does not match the current goal.");
    return;
  }

  current_plan_ = *msg;
  has_plan_ = true;
  changeState(ControllerState::TRACKING);
  RCLCPP_INFO(
    get_logger(), "Received a global path with %zu poses.",
    current_plan_.poses.size());
}

void StandaloneLocalController::goalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_goal_ = *msg;
  has_goal_ = true;

  // 기존 path의 끝점이 새 goal과 다르면 과거 path이므로 즉시 폐기한다.
  // 같은 goal에 대한 plan이 callback 순서상 먼저 도착한 경우에는 유지한다.
  if (has_plan_ && !planMatchesCurrentGoal(current_plan_)) {
    has_plan_ = false;
    current_plan_.poses.clear();
  }

  changeState(nominalState());
  RCLCPP_INFO(
    get_logger(), "Received goal: x=%.3f y=%.3f, mode=%s",
    current_goal_.pose.position.x, current_goal_.pose.position.y,
    stateName(nominalState()));
}

void StandaloneLocalController::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  has_pose_ = true;
  last_pose_received_ = now();
}

void StandaloneLocalController::scanCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg->header.frame_id.empty() && msg->header.frame_id != robot_frame_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Deskewed cloud frame is '%s', expected '%s'. Ignoring the cloud.",
      msg->header.frame_id.c_str(), robot_frame_.c_str());
    return;
  }

  if (!local_costmap_->update(*msg)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "PointCloud2 must contain float32 x/y fields.");
    return;
  }

  has_scan_ = true;
  last_scan_received_ = now();
  costmap_pub_->publish(local_costmap_->toMessage(now(), robot_frame_));
}

void StandaloneLocalController::controlLoop()
{
  const auto current_time = now();
  const bool pose_is_fresh =
    has_pose_ && (current_time - last_pose_received_).seconds() <= pose_timeout_;
  const bool scan_is_fresh =
    has_scan_ && (current_time - last_scan_received_).seconds() <= scan_timeout_;

  // 입력이 없거나 오래되면 가속도 제한을 거치지 않고 즉시 정지한다.
  const bool has_navigation_target = has_plan_ || has_goal_;
  if (!has_navigation_target || !pose_is_fresh || !scan_is_fresh) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Controller stopped: target=%d pose_fresh=%d cloud_fresh=%d",
      has_navigation_target, pose_is_fresh, scan_is_fresh);
    publishStop();
    return;
  }

  const auto target = computeVelocityCommand();
  auto command = applyAccelerationLimits(target);

  // 이전 명령과 새 명령 사이를 보간한 결과가 장애물 쪽이라면 smoothing보다
  // 안전 정지를 우선한다.
  if (trajectoryCollides(command.linear.x, command.angular.z)) {
    command = geometry_msgs::msg::Twist();
  }

  last_command_ = command;
  cmd_vel_pub_->publish(command);
}

geometry_msgs::msg::Twist
StandaloneLocalController::computeVelocityCommand()
{
  geometry_msgs::msg::Twist desired;
  const auto & robot = current_pose_.pose.pose;
  const auto & goal =
    has_plan_ ? current_plan_.poses.back().pose : current_goal_.pose;
  const ControllerState nominal_state = nominalState();
  const double goal_distance = std::hypot(
    goal.position.x - robot.position.x,
    goal.position.y - robot.position.y);

  // 최종 위치에서는 선속도를 0으로 만들고 goal yaw까지만 회전한다.
  if (goal_distance <= goal_position_tolerance_) {
    changeState(nominal_state);
    const double yaw_error = normalizeAngle(
      yawFromQuaternion(goal.orientation) -
      yawFromQuaternion(robot.orientation));
    if (std::abs(yaw_error) <= goal_yaw_tolerance_) {
      return desired;
    }

    desired.angular.z = std::clamp(
      1.5 * yaw_error, -max_angular_velocity_, max_angular_velocity_);
    return trajectoryCollides(0.0, desired.angular.z) ?
           geometry_msgs::msg::Twist() : desired;
  }

  double target_map_x = goal.position.x;
  double target_map_y = goal.position.y;

  if (has_plan_) {
    // map이 있는 정상 모드: 현재 로봇과 가장 가까운 global path point를 찾는다.
    size_t closest_index = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < current_plan_.poses.size(); ++i) {
      const auto & point = current_plan_.poses[i].pose.position;
      const double distance = std::hypot(
        point.x - robot.position.x, point.y - robot.position.y);
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_index = i;
      }
    }

    // 최근접 점부터 경로 길이를 누적하여 lookahead point를 고른다.
    size_t target_index = closest_index;
    double accumulated_distance = 0.0;
    for (size_t i = closest_index + 1; i < current_plan_.poses.size(); ++i) {
      const auto & previous = current_plan_.poses[i - 1].pose.position;
      const auto & current = current_plan_.poses[i].pose.position;
      accumulated_distance += std::hypot(
        current.x - previous.x, current.y - previous.y);
      target_index = i;
      if (accumulated_distance >= lookahead_distance_) {
        break;
      }
    }

    target_map_x = current_plan_.poses[target_index].pose.position.x;
    target_map_y = current_plan_.poses[target_index].pose.position.y;
  }

  // plan이 없으면 goal 자체가 target이고, plan이 있으면 lookahead가 target이다.
  // 어느 경우든 scan/costmap과 같은 base_link 좌표계로 변환한다.
  const auto robot_target =
    transformMapPointToRobot(target_map_x, target_map_y);
  const double target_x = robot_target.first;
  const double target_y = robot_target.second;
  const double target_distance =
    std::max(std::hypot(target_x, target_y), 1e-3);
  const double heading_error = std::atan2(target_y, target_x);

  // target이 뒤에 있으면 Pure Pursuit 곡선을 만들기 전에 제자리 회전한다.
  if (target_x <= 0.0 || std::abs(heading_error) > 1.2) {
    desired.angular.z = std::clamp(
      1.5 * heading_error, -max_angular_velocity_, max_angular_velocity_);
  } else {
    // Pure Pursuit: curvature = 2*y/L^2, angular = linear*curvature.
    const double curvature =
      2.0 * target_y / (target_distance * target_distance);
    const double heading_scale = std::clamp(
      1.0 - std::abs(heading_error) / 1.2, 0.2, 1.0);
    // 목표 0.5m 안에서는 감속하여 map 없는 직선 추종에서도 overshoot를 줄인다.
    const double goal_scale = std::clamp(goal_distance / 0.5, 0.15, 1.0);
    desired.linear.x =
      max_linear_velocity_ * heading_scale * goal_scale;
    desired.angular.z = std::clamp(
      desired.linear.x * curvature,
      -max_angular_velocity_, max_angular_velocity_);
  }

  const bool direct_path_safe =
    !trajectoryCollides(desired.linear.x, desired.angular.z);
  const double state_age = (now() - state_since_).seconds();

  if (state_ == ControllerState::TRACKING ||
    state_ == ControllerState::GOAL_SEEKING)
  {
    if (state_ != nominal_state) {
      changeState(nominal_state);
    }
    if (direct_path_safe) {
      return desired;
    }

    // 최초 충돌 시 AVOIDING으로 들어가고, 아래 sampling 결과의 회전 방향을
    // 이후 cycle에서도 유지한다.
    changeState(ControllerState::AVOIDING);
    geometry_msgs::msg::Twist avoidance;
    if (findBestAvoidanceCommand(target_x, target_y, avoidance)) {
      if (std::abs(avoidance.angular.z) > 0.05) {
        avoidance_direction_ = avoidance.angular.z > 0.0 ? 1 : -1;
      } else {
        avoidance_direction_ = chooseOpenDirection(target_y);
      }
      return avoidance;
    }

    avoidance_direction_ = chooseOpenDirection(target_y);
    changeState(ControllerState::BLOCKED);
    return blockedCommand(target_y);
  }

  // 회피 중에는 direct path가 잠깐 안전해져도 즉시 복귀하지 않는다.
  // 지정 시간과 연속 안전 cycle 조건을 모두 만족해야 TRACKING으로 돌아간다.
  if (direct_path_safe) {
    ++consecutive_safe_cycles_;
  } else {
    consecutive_safe_cycles_ = 0;
  }

  if (direct_path_safe &&
    consecutive_safe_cycles_ >= safe_cycles_required_ &&
    state_age >= avoidance_hold_time_)
  {
    changeState(nominal_state);
    return desired;
  }

  geometry_msgs::msg::Twist avoidance;
  if (findBestAvoidanceCommand(target_x, target_y, avoidance)) {
    if (state_ == ControllerState::BLOCKED) {
      changeState(ControllerState::AVOIDING);
    }
    return avoidance;
  }

  changeState(ControllerState::BLOCKED);
  return blockedCommand(target_y);
}

bool StandaloneLocalController::findBestAvoidanceCommand(
  double target_x, double target_y,
  geometry_msgs::msg::Twist & command) const
{
  double best_score = std::numeric_limits<double>::infinity();
  bool found = false;

  // 선속도와 각속도를 함께 샘플링한다. v=0은 BLOCKED에서 별도로 처리하므로
  // 여기서는 실제로 전진 가능한 후보만 평가한다.
  for (int vi = 1; vi < linear_samples_; ++vi) {
    const double linear =
      max_linear_velocity_ * static_cast<double>(vi) /
      static_cast<double>(linear_samples_ - 1);

    for (int wi = 0; wi < angular_samples_; ++wi) {
      const double ratio =
        static_cast<double>(wi) / static_cast<double>(angular_samples_ - 1);
      const double angular =
        -max_angular_velocity_ + 2.0 * max_angular_velocity_ * ratio;

      // 회피 방향이 결정된 뒤에는 반대 방향 후보를 버려 좌우 진동을 막는다.
      if (avoidance_direction_ != 0 &&
        angular * static_cast<double>(avoidance_direction_) < -1e-6)
      {
        continue;
      }

      const auto trajectory = evaluateTrajectory(linear, angular);
      if (trajectory.collision) {
        continue;
      }

      const double remaining_x = target_x - trajectory.end_x;
      const double remaining_y = target_y - trajectory.end_y;
      const double target_distance = std::hypot(remaining_x, remaining_y);
      const double heading_error = std::abs(normalizeAngle(
        std::atan2(remaining_y, remaining_x) - trajectory.end_yaw));

      // plan이 있으면 global path 전체, 없으면 현재 위치와 goal을 잇는
      // 가상 직선까지의 거리를 평가한다.
      const double reference_distance =
        distanceToReference(trajectory.end_x, trajectory.end_y);

      // minimum_clearance는 trajectory 전체에서 가장 작은 값이다.
      // 장애물에 가까울수록 reciprocal cost가 급격히 증가한다.
      const double obstacle_cost =
        1.0 / std::max(trajectory.minimum_clearance, 0.05);

      const double score =
        path_weight_ * reference_distance +
        target_weight_ * target_distance +
        heading_weight_ * heading_error +
        obstacle_weight_ * obstacle_cost -
        velocity_weight_ * linear +
        0.1 * std::abs(angular - last_command_.angular.z);

      if (score < best_score) {
        best_score = score;
        command.linear.x = linear;
        command.angular.z = angular;
        found = true;
      }
    }
  }

  return found;
}

StandaloneLocalController::TrajectoryEvaluation
StandaloneLocalController::evaluateTrajectory(
  double linear, double angular) const
{
  TrajectoryEvaluation result;
  result.minimum_clearance = std::numeric_limits<double>::infinity();

  // 현재 base_link를 (0,0,0)으로 두고 unicycle model을 적분한다.
  for (double elapsed = 0.0; elapsed <= prediction_horizon_;
    elapsed += prediction_step_)
  {
    if (local_costmap_->isCollision(result.end_x, result.end_y)) {
      result.collision = true;
      result.minimum_clearance = 0.0;
      return result;
    }

    result.minimum_clearance = std::min(
      result.minimum_clearance,
      local_costmap_->obstacleClearance(result.end_x, result.end_y));

    result.end_x +=
      linear * std::cos(result.end_yaw) * prediction_step_;
    result.end_y +=
      linear * std::sin(result.end_yaw) * prediction_step_;
    result.end_yaw = normalizeAngle(
      result.end_yaw + angular * prediction_step_);
  }

  if (local_costmap_->isCollision(result.end_x, result.end_y)) {
    result.collision = true;
    result.minimum_clearance = 0.0;
  } else {
    result.minimum_clearance = std::min(
      result.minimum_clearance,
      local_costmap_->obstacleClearance(result.end_x, result.end_y));
  }
  return result;
}

bool StandaloneLocalController::trajectoryCollides(
  double linear, double angular) const
{
  return evaluateTrajectory(linear, angular).collision;
}

double StandaloneLocalController::distanceToReference(
  double robot_x, double robot_y) const
{
  if (has_plan_) {
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto & pose : current_plan_.poses) {
      const auto point = transformMapPointToRobot(
        pose.pose.position.x, pose.pose.position.y);
      minimum = std::min(
        minimum, std::hypot(point.first - robot_x, point.second - robot_y));
    }
    return minimum;
  }

  // map 없는 fallback에서는 현재 로봇 원점과 goal을 잇는 선분을
  // 임시 reference path로 사용한다.
  const auto goal = transformMapPointToRobot(
    current_goal_.pose.position.x, current_goal_.pose.position.y);
  const double length_squared =
    goal.first * goal.first + goal.second * goal.second;
  if (length_squared <= 1e-9) {
    return std::hypot(robot_x, robot_y);
  }

  const double projection = std::clamp(
    (robot_x * goal.first + robot_y * goal.second) / length_squared,
    0.0, 1.0);
  const double nearest_x = projection * goal.first;
  const double nearest_y = projection * goal.second;
  return std::hypot(robot_x - nearest_x, robot_y - nearest_y);
}

std::pair<double, double>
StandaloneLocalController::transformMapPointToRobot(
  double map_x, double map_y) const
{
  const auto & robot = current_pose_.pose.pose;
  const double yaw = yawFromQuaternion(robot.orientation);
  const double dx = map_x - robot.position.x;
  const double dy = map_y - robot.position.y;

  // map 점에서 로봇 위치를 뺀 뒤 -robot_yaw만큼 회전한다.
  return {
    std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy
  };
}

double StandaloneLocalController::yawFromQuaternion(
  const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double StandaloneLocalController::normalizeAngle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void StandaloneLocalController::changeState(ControllerState new_state)
{
  if (state_ == new_state) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Controller state: %s -> %s",
    stateName(state_), stateName(new_state));
  state_ = new_state;
  state_since_ = now();
  consecutive_safe_cycles_ = 0;

  if (new_state == ControllerState::TRACKING ||
    new_state == ControllerState::GOAL_SEEKING)
  {
    avoidance_direction_ = 0;
  }
}

StandaloneLocalController::ControllerState
StandaloneLocalController::nominalState() const
{
  return has_plan_ ? ControllerState::TRACKING :
         ControllerState::GOAL_SEEKING;
}

const char * StandaloneLocalController::stateName(
  ControllerState state) const
{
  switch (state) {
    case ControllerState::TRACKING:
      return "TRACKING";
    case ControllerState::GOAL_SEEKING:
      return "GOAL_SEEKING";
    case ControllerState::AVOIDING:
      return "AVOIDING";
    case ControllerState::BLOCKED:
      return "BLOCKED";
  }
  return "UNKNOWN";
}

bool StandaloneLocalController::planMatchesCurrentGoal(
  const nav_msgs::msg::Path & plan) const
{
  if (!has_goal_ || plan.poses.empty()) {
    return true;
  }

  const auto & endpoint = plan.poses.back().pose.position;
  return std::hypot(
    endpoint.x - current_goal_.pose.position.x,
    endpoint.y - current_goal_.pose.position.y) <=
         plan_goal_match_tolerance_;
}

int StandaloneLocalController::chooseOpenDirection(double target_y) const
{
  // 로봇 전방의 좌/우 여러 지점을 비교하여 더 넓은 방향을 고른다.
  const double left_clearance =
    local_costmap_->obstacleClearance(0.25, 0.25) +
    local_costmap_->obstacleClearance(0.45, 0.35) +
    local_costmap_->obstacleClearance(0.65, 0.45);
  const double right_clearance =
    local_costmap_->obstacleClearance(0.25, -0.25) +
    local_costmap_->obstacleClearance(0.45, -0.35) +
    local_costmap_->obstacleClearance(0.65, -0.45);

  if (std::abs(left_clearance - right_clearance) > 0.05) {
    return left_clearance > right_clearance ? 1 : -1;
  }
  return target_y >= 0.0 ? 1 : -1;
}

geometry_msgs::msg::Twist StandaloneLocalController::blockedCommand(
  double target_y) const
{
  geometry_msgs::msg::Twist command;
  int direction = avoidance_direction_;
  if (direction == 0) {
    direction = chooseOpenDirection(target_y);
  }
  command.angular.z =
    static_cast<double>(direction) * blocked_rotate_speed_;

  // 원형 footprint 기준 제자리 회전도 불가능하면 완전히 정지한다.
  return trajectoryCollides(0.0, command.angular.z) ?
         geometry_msgs::msg::Twist() : command;
}

geometry_msgs::msg::Twist
StandaloneLocalController::applyAccelerationLimits(
  const geometry_msgs::msg::Twist & target) const
{
  // 완전 정지는 timeout/도착/충돌일 수 있으므로 즉시 반영한다.
  if (target.linear.x == 0.0 && target.angular.z == 0.0) {
    return target;
  }

  geometry_msgs::msg::Twist limited = target;
  const double dt = 1.0 / control_frequency_;
  const double linear_step = max_linear_acceleration_ * dt;
  const double angular_step = max_angular_acceleration_ * dt;

  limited.linear.x = std::clamp(
    target.linear.x,
    last_command_.linear.x - linear_step,
    last_command_.linear.x + linear_step);
  limited.angular.z = std::clamp(
    target.angular.z,
    last_command_.angular.z - angular_step,
    last_command_.angular.z + angular_step);
  return limited;
}

void StandaloneLocalController::publishStop()
{
  last_command_ = geometry_msgs::msg::Twist();
  cmd_vel_pub_->publish(last_command_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StandaloneLocalController>());
  rclcpp::shutdown();
  return 0;
}
