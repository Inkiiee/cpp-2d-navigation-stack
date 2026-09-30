#include <gtest/gtest.h>

#include "scan_match.h"
#include "scan_match_fusion.h"

#include <cmath>
#include <limits>
#include <vector>

namespace
{
using rcl_scan_match::ScanMatcher;
using rcl_scan_match_fusion::Config;
using rcl_scan_match_fusion::TrackingMode;
using rcl_scan_match_type::AlignmentQuality;
using rcl_scan_match_type::Param;
using rcl_slam_basic_type::RobotBasePose;

TEST(ScanMatchQualityTest, MeasuresAlignedAndMisalignedScans)
{
    std::vector<double> ref_x;
    std::vector<double> ref_y;
    for(int y = 0; y < 4; ++y){
        for(int x = 0; x < 5; ++x){
            ref_x.push_back(0.1 * x);
            ref_y.push_back(0.1 * y);
        }
    }
    const auto scan_x = ref_x;
    const auto scan_y = ref_y;
    ScanMatcher matcher;

    const auto aligned = matcher.evaluateAlignment(
        ref_x, ref_y, scan_x, scan_y, Param{}, 0.20);
    EXPECT_EQ(aligned.sample_count, scan_x.size());
    EXPECT_NEAR(aligned.rmse, 0.0, 1e-12);
    EXPECT_NEAR(aligned.inlier_ratio, 1.0, 1e-12);

    const auto misaligned = matcher.evaluateAlignment(
        ref_x, ref_y, scan_x, scan_y, Param{2.0, 0.0, 0.0}, 0.20);
    EXPECT_GT(misaligned.rmse, 1.0);
    EXPECT_NEAR(misaligned.inlier_ratio, 0.0, 1e-12);
}

TEST(ScanMatchQualityTest, RejectsEmptyInputWithoutNaN)
{
    ScanMatcher matcher;
    const std::vector<double> empty;
    const auto quality = matcher.evaluateAlignment(
        empty, empty, empty, empty, Param{}, 0.20);

    EXPECT_EQ(quality.sample_count, 0U);
    EXPECT_TRUE(std::isinf(quality.rmse));
    EXPECT_DOUBLE_EQ(quality.inlier_ratio, 0.0);
}

TEST(ScanMatchFusionTest, CapsGoodScanCorrectionAndFavorsOdometry)
{
    const AlignmentQuality quality{0.05, 0.90, 100};
    const RobotBasePose odom_prediction{0.0, 0.0, 0.0};
    const RobotBasePose matched_pose{0.05, -0.02, 0.03};

    const double weight = rcl_scan_match_fusion::correctionWeight(
        quality, 0.70, odom_prediction, matched_pose);
    EXPECT_GT(weight, 0.0);
    EXPECT_LE(weight, Config{}.max_scan_weight);

    const auto fused = rcl_scan_match_fusion::fuse(
        odom_prediction, matched_pose, weight);
    EXPECT_NEAR(fused.tx, matched_pose.tx * weight, 1e-12);
    EXPECT_NEAR(fused.ty, matched_pose.ty * weight, 1e-12);
    EXPECT_LT(std::abs(fused.tx), std::abs(matched_pose.tx));
}

TEST(ScanMatchFusionTest, RejectsPoorQualityAndSmoothlyDownweightsLargeInnovation)
{
    const RobotBasePose odom_prediction{0.0, 0.0, 0.0};
    const AlignmentQuality poor_quality{0.50, 0.10, 100};
    EXPECT_DOUBLE_EQ(
        rcl_scan_match_fusion::correctionWeight(
            poor_quality, 0.70, odom_prediction, RobotBasePose{0.05, 0.0, 0.0}),
        0.0);

    const AlignmentQuality good_quality{0.05, 0.90, 100};
    const double small_innovation_weight = rcl_scan_match_fusion::correctionWeight(
        good_quality, 0.70, odom_prediction, RobotBasePose{0.05, 0.0, 0.02});
    const double large_innovation_weight = rcl_scan_match_fusion::correctionWeight(
        good_quality, 0.70, odom_prediction, RobotBasePose{0.80, 0.0, 0.30});
    EXPECT_GT(small_innovation_weight, large_innovation_weight);
    EXPECT_GT(large_innovation_weight, 0.0);
}

TEST(ScanMatchFusionTest, InterpolatesHeadingAcrossAngleWrap)
{
    constexpr double pi = 3.14159265358979323846;
    const RobotBasePose odom_prediction{0.0, 0.0, pi - 0.10};
    const RobotBasePose matched_pose{0.0, 0.0, -pi + 0.10};
    const auto fused = rcl_scan_match_fusion::fuse(
        odom_prediction, matched_pose, 0.5);

    EXPECT_NEAR(std::abs(fused.theta), pi, 1e-12);
}

TEST(ScanMatchFusionTest, UsesOdometryWithoutDeclaringLostWhenOverlapIsPoor)
{
    const AlignmentQuality poor_overlap{0.70, 0.08, 100};
    const auto mode = rcl_scan_match_fusion::classifyTrackingMode(
        poor_overlap,
        0.05,
        RobotBasePose{0.0, 0.0, 0.0},
        RobotBasePose{1.0, 0.0, 0.4});

    EXPECT_EQ(mode, TrackingMode::ODOM_ONLY);
}

TEST(ScanMatchFusionTest, DeclaresLostEvidenceOnlyForObservableContradiction)
{
    const AlignmentQuality observable_match{0.22, 0.45, 100};
    const auto contradiction = rcl_scan_match_fusion::classifyTrackingMode(
        observable_match,
        0.25,
        RobotBasePose{0.0, 0.0, 0.0},
        RobotBasePose{0.45, 0.0, 0.0});
    EXPECT_EQ(contradiction, TrackingMode::LOST_EVIDENCE);

    const AlignmentQuality good_match{0.05, 0.90, 100};
    const auto normal_tracking = rcl_scan_match_fusion::classifyTrackingMode(
        good_match,
        0.70,
        RobotBasePose{0.0, 0.0, 0.0},
        RobotBasePose{0.05, 0.0, 0.02});
    EXPECT_EQ(normal_tracking, TrackingMode::FUSED);
}
}
