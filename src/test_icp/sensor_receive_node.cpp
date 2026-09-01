#include "sensor_receive_node.hpp"

#include <cmath>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"

LaserScan::LaserScan(Bridge* b, const std::string& node_name)
    : RosSubscriberNode(b, node_name, "/scan") {
    const auto deskewed_topic = declare_parameter<std::string>(
        "deskewed_scan_topic", "/scan_deskewed");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
    deskewed_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        deskewed_topic, rclcpp::SensorDataQoS());

    RCLCPP_INFO(
        get_logger(), "Deskewed scan output: topic=%s frame=%s",
        deskewed_topic.c_str(), robot_frame_.c_str());
}

void LaserScan::received(sensor_msgs::msg::LaserScan::UniquePtr msg){
    std::vector<double> xs, ys;
    const bool deskewed = bridge_->deskewScan(*msg, xs, ys);
    if(!deskewed){
        // odom 이력이 아직 없으면 scan을 버리지 않고 원본 좌표로 발행한다.
        // Local Controller는 pose/scan timeout으로 초기 구간을 안전하게 처리한다.
        double rad = msg->angle_min;
        for(auto r: msg->ranges){
            if(std::isfinite(r) && r >= msg->range_min && r <= msg->range_max){
                xs.push_back(std::cos(rad) * r);
                ys.push_back(std::sin(rad) * r);
            }
            rad += msg->angle_increment;
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "No odom history for scan deskew; publishing the raw aligned cloud.");
    }

    publishDeskewedScan(*msg, xs, ys);
    bridge_->emitScanDataReceived(xs, ys);
}

void LaserScan::publishDeskewedScan(
    const sensor_msgs::msg::LaserScan& source,
    const std::vector<double>& xs,
    const std::vector<double>& ys) {
    if(xs.size() != ys.size()){
        RCLCPP_ERROR(get_logger(), "Deskewed scan x/y sizes do not match.");
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    // deskewScan()은 각 beam을 scan 종료 시점의 로봇 좌표로 맞추므로
    // PointCloud2 timestamp도 마지막 beam 시각으로 설정한다.
    rclcpp::Time end_stamp(source.header.stamp);
    if(!source.ranges.empty()){
        end_stamp = end_stamp + rclcpp::Duration::from_seconds(
            source.time_increment * static_cast<double>(source.ranges.size() - 1));
    }
    cloud.header.stamp = end_stamp;
    cloud.header.frame_id = robot_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(xs.size());
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(xs.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for(size_t i = 0; i < xs.size(); ++i, ++iter_x, ++iter_y, ++iter_z){
        *iter_x = static_cast<float>(xs[i]);
        *iter_y = static_cast<float>(ys[i]);
        *iter_z = 0.0F;
    }

    deskewed_scan_pub_->publish(cloud);
}
