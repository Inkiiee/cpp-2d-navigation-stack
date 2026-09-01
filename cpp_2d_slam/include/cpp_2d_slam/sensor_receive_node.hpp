#ifndef __CPP_2D_SLAM__SENSOR_RECEIVE_NODE_HPP__
#define __CPP_2D_SLAM__SENSOR_RECEIVE_NODE_HPP__

#include <string>
#include <vector>

#include "ros_subscriber_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LaserScan : public RosSubscriberNode<sensor_msgs::msg::LaserScan, LaserScan> {
public:
    LaserScan(Bridge* b, const std::string& node_name = "laser_scan_reader");
    void received(sensor_msgs::msg::LaserScan::UniquePtr msg);

private:
    // deskew가 끝난 로봇 기준 점군을 Local Controller에 전달한다.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_scan_pub_;
    std::string robot_frame_;

    void publishDeskewedScan(
        const sensor_msgs::msg::LaserScan& source,
        const std::vector<double>& xs,
        const std::vector<double>& ys);
};

#endif
