#ifndef CPP_2D_SLAM__SCAN_RELOCALIZER_H_
#define CPP_2D_SLAM__SCAN_RELOCALIZER_H_

#include "scan_match.h"
#include "slam_basic.h"

#include <optional>
#include <vector>

namespace rcl_scan_relocalizer
{
struct Result
{
    rcl_scan_match_type::Param pose;
    rcl_scan_match_type::AlignmentQuality quality;
    double average_score = 0.0;
    double score_margin = 0.0;
};

class ScanRelocalizer
{
public:
    std::optional<Result> find(
        rcl_scan_match::ScanMatcher& matcher,
        const std::vector<double>& map_x,
        const std::vector<double>& map_y,
        const std::vector<double>& scan_x,
        const std::vector<double>& scan_y,
        const std::vector<rcl_slam_basic_type::RobotBasePose>& anchors) const;

    static rcl_slam_basic_type::RobotBasePose mapToOdomTransform(
        const rcl_slam_basic_type::RobotBasePose& map_pose,
        const rcl_slam_basic_type::RobotBasePose& odom_pose);

    static bool transformsAreConsistent(
        const rcl_slam_basic_type::RobotBasePose& first,
        const rcl_slam_basic_type::RobotBasePose& second,
        double max_translation = 0.25,
        double max_rotation = 0.15);
};
}

#endif  // CPP_2D_SLAM__SCAN_RELOCALIZER_H_
