#include "odom_receive_node.hpp"

OdomLoader::OdomLoader(Bridge* b, const std::string& node_name)
    : RosSubscriberNode(b, node_name, "/odom") {}

void OdomLoader::received(nav_msgs::msg::Odometry::UniquePtr msg){
    double stamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    double rx = msg->pose.pose.orientation.x;
    double ry = msg->pose.pose.orientation.y;
    double rz = msg->pose.pose.orientation.z;
    double rw = msg->pose.pose.orientation.w;

    bridge_->emitOdomDataReceived(stamp, x, y, z, rx, ry, rz, rw);
}
