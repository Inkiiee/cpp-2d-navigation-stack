#include <gtest/gtest.h>

#include "bridge.h"
#include "scan_match_backend.h"

#include <QCoreApplication>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace{
    using rcl_scan_match_backend::ScanMatchBackend;

    std::pair<ScanAxis, ScanAxis> makeRoomScan(double robot_x, double robot_y, double robot_theta){
        std::vector<std::pair<double, double>> landmarks;
        for(int i = -30; i <= 30; ++i){
            const double p = static_cast<double>(i) * 0.1;
            landmarks.emplace_back(p, -3.0);
            landmarks.emplace_back(p, 3.0);
            landmarks.emplace_back(-3.0, p);
            landmarks.emplace_back(3.0, p);
        }

        ScanAxis xs;
        ScanAxis ys;
        xs.reserve(landmarks.size());
        ys.reserve(landmarks.size());
        const double cos_theta = std::cos(robot_theta);
        const double sin_theta = std::sin(robot_theta);
        for(const auto& [world_x, world_y] : landmarks){
            const double dx = world_x - robot_x;
            const double dy = world_y - robot_y;
            xs.push_back(cos_theta * dx + sin_theta * dy);
            ys.push_back(-sin_theta * dx + cos_theta * dy);
        }
        return {xs, ys};
    }

    void processQueuedScan(){
        QCoreApplication::processEvents();
        QCoreApplication::sendPostedEvents();
        QCoreApplication::processEvents();
    }

    TEST(OdomInitializationTest, UsesFirstAbsolutePoseOnlyAsBaseline){
        Bridge bridge;
        ScanMatchBackend backend(&bridge);
        constexpr double initial_heading = 1.0;

        backend.odomUpdate(5.0, -3.0, 0.0, 0.0, 0.0, initial_heading);

        const auto initial_pose = backend.getCurrentPose();
        EXPECT_DOUBLE_EQ(initial_pose.tx, 0.0);
        EXPECT_DOUBLE_EQ(initial_pose.ty, 0.0);
        EXPECT_DOUBLE_EQ(initial_pose.theta, 0.0);

        backend.odomUpdate(
            5.0 + std::cos(initial_heading),
            -3.0 + std::sin(initial_heading),
            0.0,
            0.0,
            0.0,
            initial_heading + 0.2);

        const auto moved_pose = backend.getCurrentPose();
        EXPECT_NEAR(moved_pose.tx, 1.0, 1e-12);
        EXPECT_NEAR(moved_pose.ty, 0.0, 1e-12);
        EXPECT_NEAR(moved_pose.theta, 0.2, 1e-12);
    }

    TEST(OdomInitializationTest, IgnoresInvalidFirstSample){
        Bridge bridge;
        ScanMatchBackend backend(&bridge);

        backend.odomUpdate(
            std::numeric_limits<double>::quiet_NaN(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0);
        backend.odomUpdate(2.0, 4.0, 0.0, 0.0, 0.0, -0.5);

        const auto pose = backend.getCurrentPose();
        EXPECT_DOUBLE_EQ(pose.tx, 0.0);
        EXPECT_DOUBLE_EQ(pose.ty, 0.0);
        EXPECT_DOUBLE_EQ(pose.theta, 0.0);
    }

    TEST(OdomInitializationTest, DoesNotPullExactOdometryAwayDuringMapWarmup){
        int argc = 1;
        char application_name[] = "test_cpp_2d_slam_odom";
        char* argv[] = {application_name, nullptr};
        QCoreApplication application(argc, argv);
        Bridge bridge;
        ScanMatchBackend backend(&bridge);

        backend.odomUpdate(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        for(int i = 0; i < 3; ++i){
            auto [xs, ys] = makeRoomScan(0.0, 0.0, 0.0);
            backend.lidarUpdate(xs, ys);
            processQueuedScan();
        }

        for(int step = 1; step <= 30; ++step){
            const double x = static_cast<double>(step) * 0.02;
            backend.odomUpdate(x, 0.0, 0.0, 0.0, 0.0, 0.0);
            auto [xs, ys] = makeRoomScan(x, 0.0, 0.0);
            backend.lidarUpdate(xs, ys);
            processQueuedScan();

            const auto pose = backend.getCurrentPose();
            EXPECT_NEAR(pose.tx, x, 0.03) << "step=" << step;
            EXPECT_NEAR(pose.ty, 0.0, 0.03) << "step=" << step;
            EXPECT_NEAR(pose.theta, 0.0, 0.02) << "step=" << step;
        }
    }
}
