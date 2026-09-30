#include "scan_relocalizer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace rcl_scan_relocalizer
{
namespace
{
constexpr std::size_t kMinimumMapPoints = 30;
constexpr std::size_t kMinimumScanPoints = 20;
constexpr std::size_t kMaximumAnchorCount = 500;
constexpr int kHeadingSamples = 24;
constexpr double kMinimumInitialScore = 0.30;
constexpr double kMinimumFinalScore = 0.35;
constexpr double kMinimumScoreMargin = 0.04;
constexpr double kMaximumRmse = 0.25;
constexpr double kMinimumInlierRatio = 0.50;
constexpr double kDistinctTranslation = 0.75;
constexpr double kDistinctRotation = 0.50;
constexpr double kPi = 3.14159265358979323846;

struct Candidate
{
    rcl_slam_basic_type::RobotBasePose pose;
    double score = 0.0;
};

double poseDistance(
    const rcl_slam_basic_type::RobotBasePose& first,
    const rcl_slam_basic_type::RobotBasePose& second)
{
    return std::hypot(first.tx - second.tx, first.ty - second.ty);
}

bool isDistinct(const Candidate& first, const Candidate& second)
{
    return poseDistance(first.pose, second.pose) > kDistinctTranslation
        || std::abs(rcl_slam_basic_transform::normalizeAngle(
            first.pose.theta - second.pose.theta)) > kDistinctRotation;
}
}

std::optional<Result> ScanRelocalizer::find(
    rcl_scan_match::ScanMatcher& matcher,
    const std::vector<double>& map_x,
    const std::vector<double>& map_y,
    const std::vector<double>& scan_x,
    const std::vector<double>& scan_y,
    const std::vector<rcl_slam_basic_type::RobotBasePose>& anchors) const
{
    const std::size_t map_count = std::min(map_x.size(), map_y.size());
    const std::size_t scan_count = std::min(scan_x.size(), scan_y.size());
    if(map_count < kMinimumMapPoints
        || scan_count < kMinimumScanPoints
        || anchors.empty()){
        return std::nullopt;
    }

    std::vector<double> valid_map_x(map_x.begin(), map_x.begin() + map_count);
    std::vector<double> valid_map_y(map_y.begin(), map_y.begin() + map_count);
    std::vector<double> valid_scan_x(scan_x.begin(), scan_x.begin() + scan_count);
    std::vector<double> valid_scan_y(scan_y.begin(), scan_y.begin() + scan_count);
    const auto lookup_table = matcher.buildLookupTable(
        valid_map_x, valid_map_y, 0.05, 0.10);
    if(lookup_table.data.empty()){
        return std::nullopt;
    }

    const std::size_t anchor_stride = std::max<std::size_t>(
        1, (anchors.size() + kMaximumAnchorCount - 1) / kMaximumAnchorCount);
    std::vector<Candidate> candidates;
    candidates.reserve(
        ((anchors.size() + anchor_stride - 1) / anchor_stride) * kHeadingSamples);

    for(std::size_t anchor_index = 0; anchor_index < anchors.size(); anchor_index += anchor_stride){
        const auto& anchor = anchors[anchor_index];
        if(!std::isfinite(anchor.tx)
            || !std::isfinite(anchor.ty)
            || !std::isfinite(anchor.theta)){
            continue;
        }
        for(int heading_index = 0; heading_index < kHeadingSamples; ++heading_index){
            const double heading = -kPi
                + (2.0 * kPi * heading_index) / static_cast<double>(kHeadingSamples);
            const double score = matcher.scoreCandidate(
                lookup_table, valid_scan_x, valid_scan_y, anchor.tx, anchor.ty, heading)
                / static_cast<double>(scan_count);
            candidates.push_back(Candidate{
                rcl_slam_basic_type::RobotBasePose{anchor.tx, anchor.ty, heading}, score});
        }
    }

    if(candidates.empty()){
        return std::nullopt;
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs){
        return lhs.score > rhs.score;
    });

    const Candidate& best = candidates.front();
    if(!std::isfinite(best.score) || best.score < kMinimumInitialScore){
        return std::nullopt;
    }

    double distinct_second_score = 0.0;
    for(std::size_t i = 1; i < candidates.size(); ++i){
        if(isDistinct(best, candidates[i])){
            distinct_second_score = candidates[i].score;
            break;
        }
    }
    const double score_margin = best.score - distinct_second_score;
    if(score_margin < kMinimumScoreMargin){
        return std::nullopt;
    }

    auto refined = matcher.runCSM(
        valid_scan_x, valid_scan_y, lookup_table,
        best.pose.tx, best.pose.ty, best.pose.theta,
        0.60, 0.40,
        0.10, 0.05,
        0.02, 0.01,
        0.0, 0.0);
    refined.theta = rcl_slam_basic_transform::normalizeAngle(refined.theta);

    const double final_score = matcher.scoreCandidate(
        lookup_table, valid_scan_x, valid_scan_y,
        refined.tx, refined.ty, refined.theta) / static_cast<double>(scan_count);
    const auto quality = matcher.evaluateAlignment(
        valid_map_x, valid_map_y, valid_scan_x, valid_scan_y, refined, 0.20);
    if(!std::isfinite(final_score)
        || !std::isfinite(quality.rmse)
        || final_score < kMinimumFinalScore
        || quality.rmse > kMaximumRmse
        || quality.inlier_ratio < kMinimumInlierRatio){
        return std::nullopt;
    }

    return Result{refined, quality, final_score, score_margin};
}

rcl_slam_basic_type::RobotBasePose ScanRelocalizer::mapToOdomTransform(
    const rcl_slam_basic_type::RobotBasePose& map_pose,
    const rcl_slam_basic_type::RobotBasePose& odom_pose)
{
    const auto inverse_odom = rcl_slam_basic_transform::inversePose(odom_pose);
    const double cos_theta = std::cos(map_pose.theta);
    const double sin_theta = std::sin(map_pose.theta);
    return rcl_slam_basic_type::RobotBasePose{
        map_pose.tx + cos_theta * inverse_odom.tx - sin_theta * inverse_odom.ty,
        map_pose.ty + sin_theta * inverse_odom.tx + cos_theta * inverse_odom.ty,
        rcl_slam_basic_transform::normalizeAngle(map_pose.theta + inverse_odom.theta)};
}

bool ScanRelocalizer::transformsAreConsistent(
    const rcl_slam_basic_type::RobotBasePose& first,
    const rcl_slam_basic_type::RobotBasePose& second,
    double max_translation,
    double max_rotation)
{
    if(max_translation < 0.0 || max_rotation < 0.0){
        return false;
    }
    if(!std::isfinite(first.tx) || !std::isfinite(first.ty) || !std::isfinite(first.theta)
        || !std::isfinite(second.tx) || !std::isfinite(second.ty)
        || !std::isfinite(second.theta)){
        return false;
    }
    return poseDistance(first, second) <= max_translation
        && std::abs(rcl_slam_basic_transform::normalizeAngle(
            first.theta - second.theta)) <= max_rotation;
}
}
