#include <gtest/gtest.h>

#include "bridge.h"

#include <cstdint>

namespace{
    sensor_msgs::msg::LaserScan makeForwardScan(double stamp, double time_increment){
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp.sec = static_cast<std::int32_t>(stamp);
        scan.header.stamp.nanosec = static_cast<std::uint32_t>(
            (stamp - static_cast<double>(scan.header.stamp.sec)) * 1e9);
        scan.angle_min = 0.0F;
        scan.angle_increment = 0.0F;
        scan.time_increment = static_cast<float>(time_increment);
        scan.range_min = 0.1F;
        scan.range_max = 10.0F;
        scan.ranges = {1.0F, 1.0F};
        return scan;
    }

    TEST(BridgeDeskewTest, ClampsScanTimesNewerThanLatestOdometry){
        Bridge bridge;
        bridge.emitOdomDataReceived(10.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        bridge.emitOdomDataReceived(10.01, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

        ScanAxis xs;
        ScanAxis ys;
        const auto scan = makeForwardScan(10.02, 1.0);

        ASSERT_TRUE(bridge.deskewScan(scan, xs, ys));
        ASSERT_EQ(xs.size(), 2U);
        ASSERT_EQ(ys.size(), 2U);
        EXPECT_NEAR(xs[0], 1.0, 1e-9);
        EXPECT_NEAR(xs[1], 1.0, 1e-9);
        EXPECT_NEAR(ys[0], 0.0, 1e-9);
        EXPECT_NEAR(ys[1], 0.0, 1e-9);
    }

    TEST(BridgeDeskewTest, ClearsHistoryWhenSimulationClockMovesBackward){
        Bridge bridge;
        bridge.emitOdomDataReceived(20.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        bridge.emitOdomDataReceived(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

        ScanAxis xs;
        ScanAxis ys;
        const auto scan = makeForwardScan(1.0, 0.1);

        ASSERT_TRUE(bridge.deskewScan(scan, xs, ys));
        ASSERT_EQ(xs.size(), 2U);
        EXPECT_NEAR(xs[0], 1.0, 1e-9);
        EXPECT_NEAR(xs[1], 1.0, 1e-9);
    }
}
