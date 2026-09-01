#include "imu_receive_node.hpp"

ImuLoader::ImuLoader(Bridge* b, const std::string& node_name)
    : RosSubscriberNode(b, node_name, "/imu") {}

void ImuLoader::received(sensor_msgs::msg::Imu::UniquePtr msg){
    double stamp = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    bridge_->emitImuHeadingReceived(
        stamp,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
}
