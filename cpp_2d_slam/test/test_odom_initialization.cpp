#include <gtest/gtest.h>

#include "bridge.h"
#include "scan_match_backend.h"

#include <cmath>
#include <limits>

namespace{
    using rcl_scan_match_backend::ScanMatchBackend;

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
}
