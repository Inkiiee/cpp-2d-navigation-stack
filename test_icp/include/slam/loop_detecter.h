#ifndef __RCL_LOOP_DETECTER_H__
#define __RCL_LOOP_DETECTER_H__

#include "scan_match.h"
#include "map_backend.h"
#include "my_pose_graph.h"

#include <QObject>
#include <atomic>
#include <mutex>
#include <vector>
#include <Eigen/Dense>

namespace rcl_loop_detecter{
    class LoopDetecter: public QObject{
        Q_OBJECT
    private:
        struct Candidate{
            int index;
            double distance;
            rcl_map_backend::sub_map submap;
        };

        rcl_pose_graph::PoseGraph* pose_graph;
        rcl_scan_match::ScanMatcher scan_matcher;
        std::vector<rcl_map_backend::sub_map>* sub_maps;
        std::mutex* shared_data_mutex;
        int pending_loop_edges_ = 0;
        std::atomic<size_t> latest_requested_index_{0};  // 최신 요청 인덱스 추적

        Eigen::Matrix3d convertPoseToMatrix(const rcl_pose_graph_type::Node& node) const;
        Eigen::Matrix3d calDeltaTransform(const rcl_pose_graph_type::Node& from, const rcl_pose_graph_type::Node& to) const;
    public:
        LoopDetecter(rcl_pose_graph::PoseGraph* pg, std::vector<rcl_map_backend::sub_map>* sm, std::mutex* mutex, QObject* parent=nullptr);
        virtual ~LoopDetecter();

    public Q_SLOTS:
        void detectLoop(size_t current_index);
    Q_SIGNALS:
        void optimizedPoseUpdated(int index, const Eigen::Matrix3d& delta);
    };
}

#endif // __RCL_LOOP_DETECTER_H__
