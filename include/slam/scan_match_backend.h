#ifndef __RCL_SCAN_MATCH_BACKEND_EXECUTER_H__
#define __RCL_SCAN_MATCH_BACKEND_EXECUTER_H__

#include "scan_match.h"
#include "map_backend.h"
#include "my_pose_graph.h"
#include "bridge.h"

#include <QObject>
#include <vector>
#include <atomic>
#include <mutex>
#include <memory>
#include <Eigen/Dense>

class RosPublisherNode;

namespace rcl_scan_match_backend{
    class ScanMatchBackend: public QObject{
        Q_OBJECT
    private:
        Bridge* bridge;
        rcl_scan_match::ScanMatcher scan_matcher;
        rcl_map_backend::MapBackend world_map;
        rcl_map_backend::MapBackend local_map;
        rcl_map_backend::MapBackend match_ref_map_;  // 스캔 매칭 레퍼런스용 (매 프레임 재활용)
        std::vector<rcl_map_backend::sub_map> sub_maps;
        rcl_pose_graph::PoseGraph pose_graph;
        std::mutex shared_data_mutex_;

        std::atomic<double> map_x, map_y, map_theta, odom_x, odom_y, odom_theta, imu_theta;
        size_t sub_map_index = 0, frame_index = 0;

        // 최신 스캔 버퍼: signal은 여기에 저장만, 처리는 항상 최신값으로
        std::mutex scan_mutex_;
        ScanAxis pending_scan_x_, pending_scan_y_;
        bool has_pending_scan_ = false;
        std::atomic<bool> processing_busy_{false};

        // 거리 게이팅: 충분히 이동했을 때만 스캔 매칭
        double last_match_x_ = 0, last_match_y_ = 0, last_match_theta_ = 0;
        static constexpr double kMinTravelDistance = 0.01;  // 1cm
        static constexpr double kMinTravelAngle = 0.005;     // ~0.3도

        // CSM 캐시 & 호출 빈도 제어
        rcl_scan_match::LookupTable cached_lut_;
        bool lut_valid_ = false;
        double last_csm_x_ = 0, last_csm_y_ = 0, last_csm_theta_ = 0;

        // ROS2 퍼블리셔
        std::shared_ptr<RosPublisherNode> ros_pub_;

        // world_map reference 캐시 (매 프레임 getAdjacentPosDownsampled 회피)
        std::vector<double> cached_world_x_, cached_world_y_;
        double ref_cache_cx_ = 0, ref_cache_cy_ = 0;
        bool ref_cache_valid_ = false;
    public:
        ScanMatchBackend(Bridge* b, double pos_r=0.05, QObject* parent=nullptr);
        virtual ~ScanMatchBackend();

        void setRosPublisher(std::shared_ptr<RosPublisherNode> pub);

        std::vector<rcl_map_backend::sub_map>* getSubMaps();
        rcl_pose_graph::PoseGraph* getPoseGraph();
        rcl_map_backend::MapBackend* getWorldMap();
        rcl_map_backend::MapBackend* getLocalMap();
        std::mutex* getSharedDataMutex();
    
    public Q_SLOTS:
        void lidarUpdate(const ScanAxis& xs, const ScanAxis& ys);
    public Q_SLOTS:
        void odomUpdate(double x, double y, double z, double rx, double ry, double rz);
    public Q_SLOTS:
        void imuUpdate(double yaw);
    public Q_SLOTS:
        void poseOptimized(int index, const Eigen::Matrix3d& delta);
    
    Q_SIGNALS:
        void predictedPose(double x, double y, double theta);
    Q_SIGNALS:
        void subMapUpdated(size_t index);
    Q_SIGNALS:
        void scanUpdated(const ScanAxis& xs, const ScanAxis& ys);
    Q_SIGNALS:
        void rebuildMapRequested();
    };
}

#endif
