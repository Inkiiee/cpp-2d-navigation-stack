#include <QApplication>
#include <QTimer>
#include <thread>
#include <memory>

#include "bridge.h"
#include "imu_receive_node.hpp"
#include "slam.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_receive_node.hpp"
#include "odom_receive_node.hpp"
#include "plan_receive_node.hpp"
#include "teleopt.hpp"
#include "ros_publisher_node.hpp"

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

    rclcpp::executors::SingleThreadedExecutor ros_executor;
    ros_executor.add_node(imuLoader);
    ros_executor.add_node(odomLoader);
    ros_executor.add_node(receiver);
    ros_executor.add_node(planReceiver);
    ros_executor.add_node(ros_pub);

    std::thread t1([&ros_executor](){
        ros_executor.spin();
    });

    QTimer shutdown_timer;
    QObject::connect(&shutdown_timer, &QTimer::timeout, &app, [&app](){
        if(!rclcpp::ok()){
            app.quit();
        }
    });
    shutdown_timer.start(50);

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
    if(rclcpp::ok()){
        rclcpp::shutdown();
    }
    if(t1.joinable()){
        t1.join();
    }
    // if(t2.joinable()){
    //     t2.join();
    // }
    // if(t3.joinable()){
    //     t3.join();
    // }
    // keyInputMon->end();
    return ret;
}
