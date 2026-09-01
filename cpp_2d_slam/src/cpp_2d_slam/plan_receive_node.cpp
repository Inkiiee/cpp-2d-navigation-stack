#include "plan_receive_node.hpp"

PlanPathSubscriber::PlanPathSubscriber(Bridge* b, const std::string& node_name)
    : RosSubscriberNode(b, node_name, "/plan", rclcpp::QoS(10).reliable()) {}

void PlanPathSubscriber::received(nav_msgs::msg::Path::UniquePtr msg){
    ScanAxis xs, ys;
    xs.reserve(msg->poses.size());
    ys.reserve(msg->poses.size());

    for(const auto& stamped_pose : msg->poses){
        xs.push_back(stamped_pose.pose.position.x);
        ys.push_back(stamped_pose.pose.position.y);
    }

    bridge_->emitPlanDataReceived(xs, ys);
}
