#ifndef __CPP_2D_SLAM__ROS_SUBSCRIBER_NODE_HPP__
#define __CPP_2D_SLAM__ROS_SUBSCRIBER_NODE_HPP__

#include <memory>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "bridge.h"

template<typename MsgType, typename Derived>
class RosSubscriberNode : public rclcpp::Node {
protected:
    rclcpp::Node::SharedPtr node_;
    typename rclcpp::Subscription<MsgType>::SharedPtr sub_;
    rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
    Bridge* bridge_;

    RosSubscriberNode(Bridge* b, const std::string& node_name, const std::string& topic,
                      const rclcpp::QoS& qos = rclcpp::SensorDataQoS())
        : Node(node_name.c_str()), bridge_{b}
    {
        node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
        sub_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = sub_cb_group_;
        sub_ = node_->template create_subscription<MsgType>(
            topic,
            qos,
            std::bind(&Derived::received, static_cast<Derived*>(this), std::placeholders::_1),
            sub_options
        );
    }

public:
    virtual ~RosSubscriberNode() = default;
};

#endif
