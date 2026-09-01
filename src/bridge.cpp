#include "bridge.h"

#include <QQuaternion>
#include <QVector3D>
#include <algorithm>
#include <cmath>

namespace {
double quaternionToYaw(double rx, double ry, double rz, double rw){
    // Standard ROS quaternion-to-yaw (Z-axis rotation)
    double siny_cosp = 2.0 * (rw * rz + rx * ry);
    double cosy_cosp = 1.0 - 2.0 * (ry * ry + rz * rz);
    return std::atan2(siny_cosp, cosy_cosp);
}
}

Bridge::Bridge(QObject* parent): QObject(parent){
    qRegisterMetaType<ScanAxis>("ScanAxis");
}

double Bridge::normalizeAngle(double angle){
    while(angle > M_PI) angle -= 2.0 * M_PI;
    while(angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double Bridge::interpolateAngle(double from, double to, double ratio){
    return normalizeAngle(from + normalizeAngle(to - from) * ratio);
}

double Bridge::stampToSeconds(const builtin_interfaces::msg::Time& stamp){
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

void Bridge::trimHistoryLocked(){
    constexpr std::size_t kMaxSamples = 400;
    while(odom_history_.size() > kMaxSamples){
        odom_history_.pop_front();
    }
    while(imu_history_.size() > kMaxSamples){
        imu_history_.pop_front();
    }
}

bool Bridge::sampleYawLocked(double stamp, double& yaw) const{
    if(imu_history_.empty()){
        return false;
    }

    if(imu_history_.size() == 1 || stamp <= imu_history_.front().first){
        yaw = imu_history_.front().second;
        return true;
    }

    if(stamp >= imu_history_.back().first){
        const auto& last = imu_history_.back();
        if(imu_history_.size() < 2){
            yaw = last.second;
            return true;
        }

        const auto& prev = imu_history_[imu_history_.size() - 2];
        double dt = last.first - prev.first;
        if(dt <= 1e-6){
            yaw = last.second;
            return true;
        }

        double ratio = (stamp - prev.first) / dt;
        yaw = interpolateAngle(prev.second, last.second, ratio);
        return true;
    }

    for(std::size_t i = 1; i < imu_history_.size(); ++i){
        const auto& prev = imu_history_[i - 1];
        const auto& next = imu_history_[i];
        if(stamp > next.first){
            continue;
        }

        double dt = next.first - prev.first;
        if(dt <= 1e-6){
            yaw = next.second;
            return true;
        }

        double ratio = (stamp - prev.first) / dt;
        yaw = interpolateAngle(prev.second, next.second, ratio);
        return true;
    }

    yaw = imu_history_.back().second;
    return true;
}

bool Bridge::samplePoseLocked(double stamp, TimedPose& pose) const{
    if(odom_history_.empty()){
        return false;
    }

    if(odom_history_.size() == 1 || stamp <= odom_history_.front().stamp){
        pose = odom_history_.front();
        return true;
    }

    if(stamp >= odom_history_.back().stamp){
        const auto& last = odom_history_.back();
        if(odom_history_.size() < 2){
            pose = last;
            return true;
        }

        const auto& prev = odom_history_[odom_history_.size() - 2];
        double dt = last.stamp - prev.stamp;
        if(dt <= 1e-6){
            pose = last;
            return true;
        }

        double ratio = (stamp - prev.stamp) / dt;
        pose.stamp = stamp;
        pose.x = prev.x + (last.x - prev.x) * ratio;
        pose.y = prev.y + (last.y - prev.y) * ratio;
        pose.yaw = interpolateAngle(prev.yaw, last.yaw, ratio);
        return true;
    }

    for(std::size_t i = 1; i < odom_history_.size(); ++i){
        const auto& prev = odom_history_[i - 1];
        const auto& next = odom_history_[i];
        if(stamp > next.stamp){
            continue;
        }

        double dt = next.stamp - prev.stamp;
        if(dt <= 1e-6){
            pose = next;
            return true;
        }

        double ratio = (stamp - prev.stamp) / dt;
        pose.stamp = stamp;
        pose.x = prev.x + (next.x - prev.x) * ratio;
        pose.y = prev.y + (next.y - prev.y) * ratio;
        pose.yaw = interpolateAngle(prev.yaw, next.yaw, ratio);
        return true;
    }

    pose = odom_history_.back();
    return true;
}

void Bridge::emitScanDataReceived(const ScanAxis& xs, const ScanAxis& ys){
    emit scanDataReceived(xs, ys);
}

void Bridge::emitPlanDataReceived(const ScanAxis& xs, const ScanAxis& ys){
    emit planDataReceived(xs, ys);
}

void Bridge::emitOdomDataReceived(double stamp, double x, double y, double z, double rx, double ry, double rz, double rw){
    // Standard ROS quaternion-to-euler
    double rad_rz = quaternionToYaw(rx, ry, rz, rw);
    double siny = 2.0 * (rw * ry - rz * rx);
    double rad_ry = std::asin(std::clamp(siny, -1.0, 1.0));
    double sinr_cosp = 2.0 * (rw * rx + ry * rz);
    double cosr_cosp = 1.0 - 2.0 * (rx * rx + ry * ry);
    double rad_rx = std::atan2(sinr_cosp, cosr_cosp);

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        TimedPose pose;
        pose.stamp = stamp;
        pose.x = x;
        pose.y = y;
        pose.yaw = rad_rz;
        odom_history_.push_back(pose);
        trimHistoryLocked();
    }

    emit odomDataReceived(x, y, z, rad_rx, rad_ry, rad_rz);
}

void Bridge::emitImuHeadingReceived(double stamp, double rx, double ry, double rz, double rw){
    double yaw = quaternionToYaw(rx, ry, rz, rw);
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        imu_history_.push_back({stamp, yaw});
        trimHistoryLocked();
    }
    emit imuHeadingReceived(yaw);
}

bool Bridge::deskewScan(const sensor_msgs::msg::LaserScan& scan, ScanAxis& xs, ScanAxis& ys){
    xs.clear();
    ys.clear();

    const double start_stamp = stampToSeconds(scan.header.stamp);
    const double time_increment = scan.time_increment;
    const double end_stamp = start_stamp + time_increment * static_cast<double>(scan.ranges.empty() ? 0 : (scan.ranges.size() - 1));

    TimedPose ref_pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if(!samplePoseLocked(end_stamp, ref_pose)){
            return false;
        }
    }

    xs.reserve(scan.ranges.size());
    ys.reserve(scan.ranges.size());

    double angle = scan.angle_min;
    for(std::size_t i = 0; i < scan.ranges.size(); ++i){
        double range = scan.ranges[i];
        if(std::isfinite(range) && range >= scan.range_min && range <= scan.range_max){
            TimedPose beam_pose;
            bool has_pose = false;
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                has_pose = samplePoseLocked(start_stamp + time_increment * static_cast<double>(i), beam_pose);
            }

            double local_x = std::cos(angle) * range;
            double local_y = std::sin(angle) * range;

            if(has_pose){
                double world_x = beam_pose.x + std::cos(beam_pose.yaw) * local_x - std::sin(beam_pose.yaw) * local_y;
                double world_y = beam_pose.y + std::sin(beam_pose.yaw) * local_x + std::cos(beam_pose.yaw) * local_y;
                double dx = world_x - ref_pose.x;
                double dy = world_y - ref_pose.y;
                double ref_x = std::cos(ref_pose.yaw) * dx + std::sin(ref_pose.yaw) * dy;
                double ref_y = -std::sin(ref_pose.yaw) * dx + std::cos(ref_pose.yaw) * dy;
                xs.push_back(ref_x);
                ys.push_back(ref_y);
            } else {
                xs.push_back(local_x);
                ys.push_back(local_y);
            }
        }
        angle += scan.angle_increment;
    }

    return true;
}
