#ifndef __RCL_ROS_PUBLISHER_NODE_H__
#define __RCL_ROS_PUBLISHER_NODE_H__

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <atomic>

class RosPublisherNode : public rclcpp::Node {
public:
    explicit RosPublisherNode(const std::string& node_name = "slam_publisher")
        : Node(node_name)
    {
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(1).transient_local().reliable());
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/slam_pose", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal", 10);
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // map→odom TF + 보정된 포즈 퍼블리시 (스캔매칭 결과마다 호출)
    void publishPoseAndTF(double map_x, double map_y, double map_theta,
                          double odom_x, double odom_y, double odom_theta)
    {
        auto now = this->now();

        // ── map → odom TF 계산 ──
        // T_map_odom = T_map_base * T_odom_base^(-1)
        double dtheta = map_theta - odom_theta;
        double tx = map_x - (std::cos(dtheta) * odom_x - std::sin(dtheta) * odom_y);
        double ty = map_y - (std::sin(dtheta) * odom_x + std::cos(dtheta) * odom_y);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now;
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = tx;
        t.transform.translation.y = ty;
        t.transform.translation.z = 0.0;
        t.transform.rotation.z = std::sin(dtheta / 2.0);
        t.transform.rotation.w = std::cos(dtheta / 2.0);
        tf_broadcaster_->sendTransform(t);

        // ── /slam_pose 퍼블리시 ──
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = map_x;
        pose_msg.pose.pose.position.y = map_y;
        pose_msg.pose.pose.position.z = 0.0;
        pose_msg.pose.pose.orientation.z = std::sin(map_theta / 2.0);
        pose_msg.pose.pose.orientation.w = std::cos(map_theta / 2.0);
        pose_pub_->publish(pose_msg);
    }

    void publishGoal(double map_x, double map_y, double map_theta = 0.0)
    {
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = map_x;
        goal_msg.pose.position.y = map_y;
        goal_msg.pose.position.z = 0.0;
        goal_msg.pose.orientation.z = std::sin(map_theta / 2.0);
        goal_msg.pose.orientation.w = std::cos(map_theta / 2.0);
        goal_pub_->publish(goal_msg);
        goal_pose_pub_->publish(goal_msg);
    }

    // OccupancyGrid 맵 퍼블리시 (최소 2초 간격으로 throttle)
    void publishMap(const std::vector<int8_t>& data, int width, int height,
                    double origin_x, double origin_y, double resolution)
    {
        auto now = std::chrono::steady_clock::now();
        if (map_published_once_) {
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_map_time_).count();
            if (elapsed_ms < 2000) return;
        }
        last_map_time_ = now;
        map_published_once_ = true;

        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = this->now();
        grid.header.frame_id = "map";
        grid.info.resolution = static_cast<float>(resolution);
        grid.info.width = static_cast<uint32_t>(width);
        grid.info.height = static_cast<uint32_t>(height);
        grid.info.origin.position.x = origin_x;
        grid.info.origin.position.y = origin_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data = data;
        map_pub_->publish(grid);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::chrono::steady_clock::time_point last_map_time_;
    bool map_published_once_ = false;
};

#endif // __RCL_ROS_PUBLISHER_NODE_H__
