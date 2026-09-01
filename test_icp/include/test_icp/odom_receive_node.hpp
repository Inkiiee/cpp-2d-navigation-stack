#ifndef __TEST_ICP__ODOM_RECEIVE_NODE_HPP__
#define __TEST_ICP__ODOM_RECEIVE_NODE_HPP__

#include "ros_subscriber_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomLoader : public RosSubscriberNode<nav_msgs::msg::Odometry, OdomLoader> {
public:
    OdomLoader(Bridge* b, const std::string& node_name = "odom_loader");
    void received(nav_msgs::msg::Odometry::UniquePtr msg);
};

#endif