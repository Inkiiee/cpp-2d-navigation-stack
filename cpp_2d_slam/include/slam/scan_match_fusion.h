#ifndef CPP_2D_SLAM__SCAN_MATCH_FUSION_H_
#define CPP_2D_SLAM__SCAN_MATCH_FUSION_H_

#include "scan_match.h"
#include "slam_basic.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rcl_scan_match_fusion
{
struct Config
{
    double max_rmse = 0.30;
    double min_inlier_ratio = 0.35;
    double min_csm_avg_score = 0.18;
    std::size_t min_samples = 10;

    double good_rmse = 0.08;
    double good_inlier_ratio = 0.75;
    double good_csm_avg_score = 0.55;
    double max_scan_weight = 0.35;
    double correction_distance_scale = 0.20;
    double correction_angle_scale = 0.12;
};

inline bool isFinitePose(const rcl_slam_basic_type::RobotBasePose& pose)
{
    return std::isfinite(pose.tx) && std::isfinite(pose.ty) && std::isfinite(pose.theta);
}

inline bool passesQualityGate(
    const rcl_scan_match_type::AlignmentQuality& quality,
    double csm_avg_score,
    const Config& config = Config{})
{
    if(!std::isfinite(quality.rmse)
        || !std::isfinite(quality.inlier_ratio)
        || quality.sample_count < config.min_samples
        || quality.rmse > config.max_rmse
        || quality.inlier_ratio < config.min_inlier_ratio){
        return false;
    }

    return !std::isfinite(csm_avg_score) || csm_avg_score >= config.min_csm_avg_score;
}

inline double normalizedConfidence(double value, double bad, double good)
{
    if(good == bad){
        return value >= good ? 1.0 : 0.0;
    }
    return std::clamp((value - bad) / (good - bad), 0.0, 1.0);
}

inline double correctionWeight(
    const rcl_scan_match_type::AlignmentQuality& quality,
    double csm_avg_score,
    const rcl_slam_basic_type::RobotBasePose& odom_prediction,
    const rcl_slam_basic_type::RobotBasePose& matched_pose,
    const Config& config = Config{})
{
    if(!isFinitePose(odom_prediction)
        || !isFinitePose(matched_pose)
        || !passesQualityGate(quality, csm_avg_score, config)){
        return 0.0;
    }

    const double rmse_confidence = normalizedConfidence(
        quality.rmse, config.max_rmse, config.good_rmse);
    const double inlier_confidence = normalizedConfidence(
        quality.inlier_ratio, config.min_inlier_ratio, config.good_inlier_ratio);

    double quality_confidence = std::min(rmse_confidence, inlier_confidence);
    if(std::isfinite(csm_avg_score)){
        quality_confidence = std::min(
            quality_confidence,
            normalizedConfidence(
                csm_avg_score, config.min_csm_avg_score, config.good_csm_avg_score));
    }

    const double dx = matched_pose.tx - odom_prediction.tx;
    const double dy = matched_pose.ty - odom_prediction.ty;
    const double dtheta = rcl_slam_basic_transform::normalizeAngle(
        matched_pose.theta - odom_prediction.theta);
    const double distance_ratio = std::hypot(dx, dy) / config.correction_distance_scale;
    const double angle_ratio = std::abs(dtheta) / config.correction_angle_scale;
    const double innovation_scale = 1.0 /
        (1.0 + distance_ratio * distance_ratio + angle_ratio * angle_ratio);

    return std::clamp(
        config.max_scan_weight * quality_confidence * innovation_scale,
        0.0,
        config.max_scan_weight);
}

inline rcl_slam_basic_type::RobotBasePose fuse(
    const rcl_slam_basic_type::RobotBasePose& odom_prediction,
    const rcl_slam_basic_type::RobotBasePose& matched_pose,
    double scan_weight)
{
    const double weight = std::clamp(scan_weight, 0.0, 1.0);
    const double dtheta = rcl_slam_basic_transform::normalizeAngle(
        matched_pose.theta - odom_prediction.theta);
    return rcl_slam_basic_type::RobotBasePose{
        odom_prediction.tx + weight * (matched_pose.tx - odom_prediction.tx),
        odom_prediction.ty + weight * (matched_pose.ty - odom_prediction.ty),
        rcl_slam_basic_transform::normalizeAngle(odom_prediction.theta + weight * dtheta)};
}
}

#endif  // CPP_2D_SLAM__SCAN_MATCH_FUSION_H_
