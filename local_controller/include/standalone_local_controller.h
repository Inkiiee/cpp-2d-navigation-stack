#ifndef LOCAL_CONTROLLER__STANDALONE_LOCAL_CONTROLLER_H_
#define LOCAL_CONTROLLER__STANDALONE_LOCAL_CONTROLLER_H_

#include <memory>
#include <string>
#include <utility>

#include "local_costmap.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class StandaloneLocalController : public rclcpp::Node
{
public:
  StandaloneLocalController();

private:
  enum class ControllerState
  {
    TRACKING,
    GOAL_SEEKING,
    AVOIDING,
    BLOCKED
  };

  struct TrajectoryEvaluation
  {
    bool collision = false;
    double end_x = 0.0;
    double end_y = 0.0;
    double end_yaw = 0.0;
    double minimum_clearance = 0.0;
  };

  // 입출력 토픽과 Local Costmap이 따라갈 로봇 좌표계 이름.
  std::string plan_topic_;
  std::string pose_topic_;
  std::string goal_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string costmap_topic_;
  std::string robot_frame_;

  // Pure Pursuit와 목표 도착 판정에 사용하는 제어 파라미터.
  double control_frequency_ = 20.0;
  double lookahead_distance_ = 0.4;
  double max_linear_velocity_ = 0.2;
  double max_angular_velocity_ = 0.8;
  double goal_position_tolerance_ = 0.1;
  double goal_yaw_tolerance_ = 0.15;
  double plan_goal_match_tolerance_ = 0.75;

  // 센서 안전 정지와 미래 궤적 시뮬레이션 파라미터.
  double pose_timeout_ = 0.5;
  double scan_timeout_ = 0.5;
  double prediction_horizon_ = 1.5;
  double prediction_step_ = 0.1;
  double avoidance_hold_time_ = 0.8;
  double blocked_rotate_speed_ = 0.3;
  double max_linear_acceleration_ = 0.3;
  double max_angular_acceleration_ = 1.5;
  double path_weight_ = 2.0;
  double target_weight_ = 1.0;
  double heading_weight_ = 1.0;
  double obstacle_weight_ = 0.4;
  double velocity_weight_ = 0.5;
  int linear_samples_ = 5;
  int angular_samples_ = 21;
  int safe_cycles_required_ = 5;

  // 필요한 입력을 한 번이라도 받았는지 나타내는 상태.
  bool has_plan_ = false;
  bool has_goal_ = false;
  bool has_pose_ = false;
  bool has_scan_ = false;

  // 가장 최근에 받은 global path, map 기준 pose, robot 기준 costmap.
  nav_msgs::msg::Path current_plan_;
  geometry_msgs::msg::PoseStamped current_goal_;
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
  std::unique_ptr<LocalCostmap> local_costmap_;
  rclcpp::Time last_pose_received_;
  rclcpp::Time last_scan_received_;
  rclcpp::Time state_since_;
  ControllerState state_ = ControllerState::TRACKING;
  int avoidance_direction_ = 0;
  int consecutive_safe_cycles_ = 0;
  geometry_msgs::msg::Twist last_command_;

  // ROS 입출력 객체와 주기적으로 실행되는 제어 타이머.
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  void planCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void controlLoop();

  // Pure Pursuit 기본 속도를 계산한 뒤 충돌 회피 결과를 반환한다.
  geometry_msgs::msg::Twist computeVelocityCommand();

  // 선속도와 각속도를 함께 샘플링하고 가장 점수가 낮은 궤적을 고른다.
  bool findBestAvoidanceCommand(
    double target_x, double target_y,
    geometry_msgs::msg::Twist & command) const;

  // 차동구동 운동 모델로 미래 궤적 전체를 평가한다.
  TrajectoryEvaluation evaluateTrajectory(
    double linear, double angular) const;
  bool trajectoryCollides(double linear, double angular) const;
  double distanceToReference(double robot_x, double robot_y) const;

  // map 좌표의 경로 점을 현재 로봇 중심 좌표로 변환한다.
  std::pair<double, double> transformMapPointToRobot(
    double map_x, double map_y) const;
  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  double normalizeAngle(double angle) const;

  void changeState(ControllerState new_state);
  ControllerState nominalState() const;
  const char * stateName(ControllerState state) const;
  bool planMatchesCurrentGoal(const nav_msgs::msg::Path & plan) const;
  int chooseOpenDirection(double target_y) const;
  geometry_msgs::msg::Twist blockedCommand(double target_y) const;
  geometry_msgs::msg::Twist applyAccelerationLimits(
    const geometry_msgs::msg::Twist & target) const;

  // 모든 Twist 성분이 0인 안전 정지 명령을 발행한다.
  void publishStop();
};

#endif  // LOCAL_CONTROLLER__STANDALONE_LOCAL_CONTROLLER_H_
