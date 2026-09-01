#ifndef __RCL_ROS_QT_BRIDGE_H__
#define __RCL_ROS_QT_BRIDGE_H__

#include <QObject>
#include <QMetaType>
#include <deque>
#include <mutex>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

using ScanAxis = std::vector<double>;

Q_DECLARE_METATYPE(ScanAxis)

class Bridge: public QObject{
    Q_OBJECT
public:
    struct TimedPose{
        double stamp = 0.0;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    Bridge(QObject* parent=nullptr);
    virtual ~Bridge() = default;

    void emitScanDataReceived(const ScanAxis& xs, const ScanAxis& ys);
    void emitPlanDataReceived(const ScanAxis& xs, const ScanAxis& ys);
    void emitOdomDataReceived(double stamp, double x, double y, double z, double rx, double ry, double rz, double rw);
    void emitImuHeadingReceived(double stamp, double rx, double ry, double rz, double rw);
    bool deskewScan(const sensor_msgs::msg::LaserScan& scan, ScanAxis& xs, ScanAxis& ys);

private:
    mutable std::mutex pose_mutex_;
    std::deque<TimedPose> odom_history_;
    std::deque<std::pair<double, double>> imu_history_;

    static double normalizeAngle(double angle);
    static double interpolateAngle(double from, double to, double ratio);
    static double stampToSeconds(const builtin_interfaces::msg::Time& stamp);
    void trimHistoryLocked();
    bool samplePoseLocked(double stamp, TimedPose& pose) const;
    bool sampleYawLocked(double stamp, double& yaw) const;

Q_SIGNALS:
    void scanDataReceived(const ScanAxis& xs, const ScanAxis& ys);
Q_SIGNALS:
    void planDataReceived(const ScanAxis& xs, const ScanAxis& ys);
Q_SIGNALS:
    void odomDataReceived(double x, double y, double z, double rx, double ry, double rz);
Q_SIGNALS:
    void imuHeadingReceived(double yaw);
};

#endif // __RCL_ROS_QT_BRIDGE_H__
