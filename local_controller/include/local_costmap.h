#ifndef LOCAL_CONTROLLER__LOCAL_COSTMAP_H_
#define LOCAL_CONTROLLER__LOCAL_COSTMAP_H_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LocalCostmap
{
public:
  // 로봇 중심의 고정 크기 격자를 생성한다.
  // robot_radius와 safety_margin을 합친 거리만큼 장애물을 팽창시킨다.
  LocalCostmap(
    double width, double height, double resolution,
    double robot_radius, double safety_margin, double obstacle_max_range);

  // cpp_2d_slam이 deskew한 최신 PointCloud2 한 장으로 코스트맵을 갱신한다.
  // x/y 필드가 없으면 false를 반환한다.
  bool update(const sensor_msgs::msg::PointCloud2 & cloud);

  // 로봇 좌표계의 (x, y)에 로봇 중심을 놓았을 때 충돌하는지 반환한다.
  bool isCollision(double robot_x, double robot_y) const;

  // (x, y)에서 가장 가까운 장애물까지의 여유 거리를 반환한다.
  double obstacleClearance(double robot_x, double robot_y) const;

  // RViz에서 확인할 수 있도록 내부 격자를 OccupancyGrid로 변환한다.
  nav_msgs::msg::OccupancyGrid toMessage(
    const rclcpp::Time & stamp, const std::string & frame_id) const;

private:
  // 실제 격자 크기와 해상도. 단위는 meter, meter/cell이다.
  double width_;
  double height_;
  double resolution_;
  double inflation_radius_;
  double obstacle_max_range_;
  int cells_x_;
  int cells_y_;

  // grid_: 0=주행 가능, 100=장애물 또는 inflation 영역.
  // clearance_grid_: 각 셀에서 inflation 영역까지의 최단거리(m).
  std::vector<int8_t> grid_;
  std::vector<double> clearance_grid_;

  // 로봇 좌표계의 meter 위치를 격자 인덱스로 변환한다.
  bool worldToGrid(double x, double y, int & gx, int & gy) const;
  int index(int gx, int gy) const;

  // 한 scan endpoint 주변을 로봇 반경만큼 장애물로 표시한다.
  void markInflatedObstacle(double x, double y);

  // 모든 trajectory 후보가 O(1)로 여유 거리를 조회하도록 거리장을 만든다.
  void buildClearanceField();
};

#endif  // LOCAL_CONTROLLER__LOCAL_COSTMAP_H_
