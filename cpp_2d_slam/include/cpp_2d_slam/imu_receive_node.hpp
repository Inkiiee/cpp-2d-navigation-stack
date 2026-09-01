#ifndef __CPP_2D_SLAM__IMU_RECEIVE_NODE_HPP__
#define __CPP_2D_SLAM__IMU_RECEIVE_NODE_HPP__

#include "ros_subscriber_node.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuLoader : public RosSubscriberNode<sensor_msgs::msg::Imu, ImuLoader> {
public:
    explicit ImuLoader(Bridge* b, const std::string& node_name = "imu_loader");
    void received(sensor_msgs::msg::Imu::UniquePtr msg);
};

#endif
