#ifndef __RCL_POSE_GRAPH_H__
#define __RCL_POSE_GRAPH_H__  

#include <mutex>
#include <vector>
#include <Eigen/Dense>

#include "slam_basic.h"

namespace rcl_pose_graph_type{
    struct Node: public rcl_slam_basic_type::RobotBasePose{
        Node(double xx=0, double yy=0, double t=0);
        Node(const Node& n);
        Node& operator=(const Node&) = default;
        Node(const rcl_slam_basic_type::RobotBasePose& p);
    };
    struct Edge{
        int from, to;
        double info_tx, info_ty, info_theta;
        Node relative_pose;
        bool is_loop;

        Edge(int f, int t, double itx, double ity, double itheta, bool loop=false);
        void set_relative_pose(const Node& n);
    };
}

namespace rcl_pose_graph{
    class PoseGraph{
    private:
        std::vector<rcl_pose_graph_type::Node> pose_history;
        std::vector<rcl_pose_graph_type::Edge> edges;
        mutable std::mutex mutex_;

        Eigen::Vector3d errorComputeUnlocked(const rcl_pose_graph_type::Edge& edge) const;
    public:
        void addPose(double x, double y, double theta);
        void addPose(const rcl_slam_basic_type::RobotBasePose& pose);
        void addEdge(int from, int to, double info_tx, double info_ty, double info_theta, bool is_loop=false);
        void addEdge(const rcl_pose_graph_type::Edge& edge);
        size_t getPoseCount() const;
        rcl_slam_basic_type::RobotBasePose getPose(int index) const;
        void setPose(int index, const rcl_slam_basic_type::RobotBasePose& pose);

        // Bulk 조회: lock 1회로 전체 스냅샷
        std::vector<rcl_pose_graph_type::Node> getPoseSnapshot() const;
        std::vector<rcl_pose_graph_type::Node> getPoseSnapshot(int from, int to) const;

        // Bulk 업데이트: lock 1회로 범위 내 포즈 일괄 설정
        void setPoses(int from, const std::vector<rcl_slam_basic_type::RobotBasePose>& poses);

        // Pose graph optimization 관련 함수들
        Eigen::Vector3d errorCompute(const rcl_pose_graph_type::Edge& edge) const;
        void loopOptimize(int iter=20, double epsilon=1e-6);
    };
}

#endif // __RCL_POSE_GRAPH_H__
