#ifndef __TEST_ICP__PLAN_RECEIVE_NODE_HPP__
#define __TEST_ICP__PLAN_RECEIVE_NODE_HPP__

#include "ros_subscriber_node.hpp"
#include "nav_msgs/msg/path.hpp"

class PlanPathSubscriber : public RosSubscriberNode<nav_msgs::msg::Path, PlanPathSubscriber> {
public:
    PlanPathSubscriber(Bridge* b, const std::string& node_name = "plan_path_reader");
    void received(nav_msgs::msg::Path::UniquePtr msg);
};

#endif
