#include <gtest/gtest.h>

#include "scan_relocalizer.h"

#include <cmath>
#include <vector>

namespace
{
using rcl_scan_match::ScanMatcher;
using rcl_scan_relocalizer::ScanRelocalizer;
using rcl_slam_basic_type::RobotBasePose;

void appendAsymmetricRoom(
    double offset_x,
    std::vector<double>& map_x,
    std::vector<double>& map_y)
{
    for(int i = 0; i <= 30; ++i){
        map_x.push_back(offset_x + 0.10 * i);
        map_y.push_back(0.0);
    }
    for(int i = 1; i <= 20; ++i){
        map_x.push_back(offset_x);
        map_y.push_back(0.10 * i);
    }
    for(int i = 1; i <= 12; ++i){
        map_x.push_back(offset_x + 1.4 + 0.05 * i);
        map_y.push_back(0.6 + 0.10 * i);
    }
}

void mapPointsToScan(
    const std::vector<double>& map_x,
    const std::vector<double>& map_y,
    const RobotBasePose& pose,
    std::vector<double>& scan_x,
    std::vector<double>& scan_y)
{
    const double cos_theta = std::cos(pose.theta);
    const double sin_theta = std::sin(pose.theta);
    scan_x.clear();
    scan_y.clear();
    for(std::size_t i = 0; i < map_x.size(); ++i){
        const double dx = map_x[i] - pose.tx;
        const double dy = map_y[i] - pose.ty;
        scan_x.push_back(cos_theta * dx + sin_theta * dy);
        scan_y.push_back(-sin_theta * dx + cos_theta * dy);
    }
}

TEST(ScanRelocalizerTest, FindsUniqueGlobalPose)
{
    std::vector<double> map_x;
    std::vector<double> map_y;
    appendAsymmetricRoom(0.0, map_x, map_y);
    const RobotBasePose expected_pose{1.0, 0.5, 0.0};
    std::vector<double> scan_x;
    std::vector<double> scan_y;
    mapPointsToScan(map_x, map_y, expected_pose, scan_x, scan_y);

    ScanMatcher matcher;
    ScanRelocalizer relocalizer;
    const std::vector<RobotBasePose> anchors{
        expected_pose,
        RobotBasePose{6.0, 5.0, 0.0}};
    const auto result = relocalizer.find(
        matcher, map_x, map_y, scan_x, scan_y, anchors);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->pose.tx, expected_pose.tx, 0.05);
    EXPECT_NEAR(result->pose.ty, expected_pose.ty, 0.05);
    EXPECT_NEAR(result->pose.theta, expected_pose.theta, 0.03);
    EXPECT_GT(result->average_score, 0.80);
    EXPECT_GT(result->quality.inlier_ratio, 0.90);
}

TEST(ScanRelocalizerTest, RejectsAmbiguousRepeatedRooms)
{
    std::vector<double> first_x;
    std::vector<double> first_y;
    appendAsymmetricRoom(0.0, first_x, first_y);
    std::vector<double> map_x = first_x;
    std::vector<double> map_y = first_y;
    appendAsymmetricRoom(5.0, map_x, map_y);

    const RobotBasePose first_room_pose{1.0, 0.5, 0.0};
    std::vector<double> scan_x;
    std::vector<double> scan_y;
    mapPointsToScan(first_x, first_y, first_room_pose, scan_x, scan_y);

    ScanMatcher matcher;
    ScanRelocalizer relocalizer;
    const std::vector<RobotBasePose> anchors{
        first_room_pose,
        RobotBasePose{6.0, 0.5, 0.0}};
    const auto result = relocalizer.find(
        matcher, map_x, map_y, scan_x, scan_y, anchors);

    EXPECT_FALSE(result.has_value());
}

TEST(ScanRelocalizerTest, ComparesMapToOdomTransformsAcrossRobotMotion)
{
    const auto first = ScanRelocalizer::mapToOdomTransform(
        RobotBasePose{2.0, 3.0, 0.2},
        RobotBasePose{1.0, 1.0, 0.2});
    const auto second = ScanRelocalizer::mapToOdomTransform(
        RobotBasePose{2.1, 3.0, 0.2},
        RobotBasePose{1.1, 1.0, 0.2});

    EXPECT_TRUE(ScanRelocalizer::transformsAreConsistent(first, second, 0.02, 0.01));
    EXPECT_FALSE(ScanRelocalizer::transformsAreConsistent(
        first, RobotBasePose{first.tx + 1.0, first.ty, first.theta}, 0.25, 0.15));
}
}
