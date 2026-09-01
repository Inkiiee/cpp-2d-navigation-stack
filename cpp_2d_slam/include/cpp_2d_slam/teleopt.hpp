#ifndef __TELEOPT_HPP__
#define __TELEOPT_HPP__

#include <atomic>
#include <chrono>
#include <memory>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class SharedMem{
private:
    std::atomic<double> speed_;
    std::atomic<double> theta_;
public:
    SharedMem();
    double get_theta();
    double get_speed();
    void set_theta(double t);
    void set_speed(double s);
};

class MyTelNode : public rclcpp::Node{
private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::CallbackGroup::SharedPtr publihser_cb_group_;
    std::shared_ptr<SharedMem> mem_;

    void timer_cb();
public:
    explicit MyTelNode(const std::shared_ptr<SharedMem>& m);
    virtual ~MyTelNode() = default;
};

class KeyInputMon{
    std::shared_ptr<SharedMem> mem_;
    struct termios old_setting_;
    bool is_end_;
public:
    KeyInputMon(const std::shared_ptr<SharedMem>& m);
    void process();
    void end();
    ~KeyInputMon();
};

#endif // __TELEOPT_HPP__