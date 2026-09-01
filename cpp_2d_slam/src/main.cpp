#include <QApplication>
#include <thread>
#include <memory>
#include <csignal>

#include "bridge.h"
#include "imu_receive_node.hpp"
#include "slam.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_receive_node.hpp"
#include "odom_receive_node.hpp"
#include "plan_receive_node.hpp"
#include "teleopt.hpp"
#include "ros_publisher_node.hpp"

bool is_end = false;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        is_end = true;
        rclcpp::shutdown();
    }
}

int main(int argc, char *argv[])
{
    //rclcpp 초기화
    rclcpp::init(argc, argv);

    //Qt 초기화
    std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
    std::vector<char *> non_ros_args_c_strings;
    for (auto & arg : non_ros_args)
        non_ros_args_c_strings.push_back(&arg.front());
    int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());
    QApplication app(non_ros_argc, non_ros_args_c_strings.data());

    std::signal(SIGINT, signal_handler);

    Bridge bridge;
    auto ros_pub = std::make_shared<RosPublisherNode>();
    auto slam_system = std::make_unique<rcl_slam::SlamSystem>(&bridge, ros_pub);

    //rcl 루프 실행 및 브리지 등록
    auto receiver = std::make_shared<LaserScan>(&bridge, "my_laser_scan_node");
    auto odomLoader = std::make_shared<OdomLoader>(&bridge);
    auto imuLoader = std::make_shared<ImuLoader>(&bridge);
    auto planReceiver = std::make_shared<PlanPathSubscriber>(&bridge);

    auto sharedMemPtr = std::make_shared<SharedMem>();
    // auto keyInputMon = std::make_shared<KeyInputMon>(sharedMemPtr);
    // auto myTelNode = std::make_shared<MyTelNode>(sharedMemPtr);

    std::thread t1([odomLoader, receiver, imuLoader, planReceiver, ros_pub](){
        while(!is_end){
            rclcpp::spin_some(imuLoader);
            rclcpp::spin_some(odomLoader);
            rclcpp::spin_some(receiver);
            rclcpp::spin_some(planReceiver);
            rclcpp::spin_some(ros_pub);
        }
    });

    // std::thread t2([keyInputMon](){
    //     keyInputMon->process();
    // });

    // std::thread t3([myTelNode](){
    //     while(!is_end){
    //         rclcpp::spin_some(myTelNode);
    //     }
    // });
    // slam_system->setSharedMem(sharedMemPtr.get());

    int ret = app.exec();

    //종료.
    is_end = true;
    if(t1.joinable()){
        t1.join();
    }
    // if(t2.joinable()){
    //     t2.join();
    // }
    // if(t3.joinable()){
    //     t3.join();
    // }
    rclcpp::shutdown();
    // keyInputMon->end();
    return ret;
}
