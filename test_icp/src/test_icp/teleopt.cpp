#include "teleopt.hpp"

SharedMem::SharedMem(): speed_{0.0}, theta_{0.0}{}
double SharedMem::get_theta(){
    return theta_.load();
}
double SharedMem::get_speed(){
    return speed_.load();
}
void SharedMem::set_theta(double t){
    theta_.store(t);
}
void SharedMem::set_speed(double s){
    speed_.store(s);
}

KeyInputMon::KeyInputMon(const std::shared_ptr<SharedMem>& m): mem_{m}, is_end_{false}{
    tcgetattr(STDIN_FILENO, &old_setting_);
    
    struct termios new_setting = old_setting_;
    new_setting.c_lflag &=~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_setting);
}

void KeyInputMon::process(){
    static double step = 0.01;
    static double theta_step = 0.1;

    char ch = ' ';
    while((ch = getchar()) != 'q' && !is_end_){
        if(ch == 'w' || ch == 'W'){
            double val = mem_->get_speed();
            if(val <= 0.22)
                mem_->set_speed(val + step);
        }
        else if(ch == 's' || ch == 'S'){
            mem_->set_speed(0.0);
            mem_->set_theta(0.0);
        }
        else if(ch == 'x' || ch == 'X'){
            double val = mem_->get_speed();
            if(val >= -0.22)
                mem_->set_speed(val - step);
        }
        else if(ch == 'a' || ch == 'A'){
            double val = mem_->get_theta();
            if(val <= 2.84)
                mem_->set_theta(val + theta_step);
        }
        else if(ch == 'd' || ch == 'D'){
            double val = mem_->get_theta();
            if(val >= -2.84)
                mem_->set_theta(val - theta_step);
        }
        else
            continue;
    }
}
void KeyInputMon::end(){
    is_end_ = true;
}
KeyInputMon::~KeyInputMon(){
    tcsetattr(STDIN_FILENO, TCSANOW, &old_setting_);
}

MyTelNode::MyTelNode(const std::shared_ptr<SharedMem>& m): Node("my_teleop_keyboard"), mem_{m}{
    this->node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *){});
    
    this->publihser_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions publisher_opts;
    publisher_opts.callback_group = this->publihser_cb_group_;
    publisher_ = this->node_->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
        publisher_opts
    );

    // auto timer_callback = [this](){
        
    // };
    
    this->timer_cb_group_ = this->node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->node_->create_wall_timer(
        std::chrono::milliseconds(118),
        std::bind(&MyTelNode::timer_cb, this),
        this->timer_cb_group_
    );
}

void MyTelNode::timer_cb(){
    auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = mem_->get_speed();
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = mem_->get_theta();

        publisher_->publish(msg);
}