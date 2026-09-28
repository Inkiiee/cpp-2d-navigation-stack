#include <gtest/gtest.h>

#include "scan_match.h"

#include <cmath>
#include <limits>
#include <vector>

namespace{
    using rcl_scan_match::ScanMatcher;
    using rcl_scan_match_type::Param;

    TEST(ScanMatchQualityTest, AcceptsAlignedScan){
        ScanMatcher matcher;
        std::vector<double> reference_x{0.0, 1.0, 2.0, 3.0};
        std::vector<double> reference_y{0.0, 0.0, 0.0, 0.0};
        std::vector<double> scan_x = reference_x;
        std::vector<double> scan_y = reference_y;

        const auto quality = matcher.evaluateMatchQuality(
            reference_x, reference_y, scan_x, scan_y, Param{}, 0.20, 0.35, 0.30);

        EXPECT_TRUE(quality.accepted);
        EXPECT_NEAR(quality.rmse, 0.0, 1e-12);
        EXPECT_NEAR(quality.inlier_ratio, 1.0, 1e-12);
    }

    TEST(ScanMatchQualityTest, RejectsPoorOverlap){
        ScanMatcher matcher;
        std::vector<double> reference_x{0.0, 1.0, 2.0, 3.0};
        std::vector<double> reference_y{0.0, 0.0, 0.0, 0.0};
        std::vector<double> scan_x{0.0, 1.0, 2.0, 3.0};
        std::vector<double> scan_y{2.0, 2.0, 2.0, 2.0};

        const auto quality = matcher.evaluateMatchQuality(
            reference_x, reference_y, scan_x, scan_y, Param{}, 0.20, 0.35, 0.30);

        EXPECT_FALSE(quality.accepted);
        EXPECT_GT(quality.rmse, 0.35);
        EXPECT_LT(quality.inlier_ratio, 0.30);
    }

    TEST(ScanMatchQualityTest, ScoresEveryScanPointAgainstSparseReference){
        ScanMatcher matcher;
        std::vector<double> reference_x{0.0};
        std::vector<double> reference_y{0.0};
        std::vector<double> scan_x{0.0, 2.0};
        std::vector<double> scan_y{0.0, 0.0};

        const auto quality = matcher.evaluateMatchQuality(
            reference_x, reference_y, scan_x, scan_y, Param{}, 0.20, 0.35, 0.75);

        EXPECT_FALSE(quality.accepted);
        EXPECT_NEAR(quality.rmse, std::sqrt(2.0), 1e-12);
        EXPECT_NEAR(quality.inlier_ratio, 0.5, 1e-12);
    }

    TEST(ScanMatchQualityTest, RejectsInvalidInput){
        ScanMatcher matcher;
        std::vector<double> reference_x{0.0};
        std::vector<double> reference_y{0.0};
        std::vector<double> scan_x{0.0};
        std::vector<double> scan_y{0.0};
        Param pose;
        pose.tx = std::numeric_limits<double>::quiet_NaN();

        const auto quality = matcher.evaluateMatchQuality(
            reference_x, reference_y, scan_x, scan_y, pose, 0.20, 0.35, 0.30);

        EXPECT_FALSE(quality.accepted);
        EXPECT_FALSE(std::isfinite(quality.rmse));
        EXPECT_DOUBLE_EQ(quality.inlier_ratio, 0.0);
    }
}
