#include "my_pose_graph.h"
#include "slam_basic.h"

#include <cmath>
#include <algorithm>
#include <Eigen/Sparse>
#ifdef TEST_ICP_HAS_G2O
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/types_slam2d.h>
#endif

namespace rcl_pose_graph_type{
    using rcl_slam_basic_type::RobotBasePose;

    Node::Node(double xx, double yy, double t): RobotBasePose(xx, yy, t) {}
    Node::Node(const Node& n): RobotBasePose(n.tx, n.ty, n.theta) {}
    Node::Node(const RobotBasePose& p): RobotBasePose(p.tx, p.ty, p.theta) {}

    Edge::Edge(int f, int t, double itx, double ity, double itheta, bool loop):
        from(f), to(t), info_tx(itx), info_ty(ity), info_theta(itheta), is_loop(loop){}
    void Edge::set_relative_pose(const Node& n){
        relative_pose.tx = n.tx;
        relative_pose.ty = n.ty;
        relative_pose.theta = n.theta;
    }
}

namespace rcl_pose_graph{
    using namespace rcl_slam_basic_type;
    using namespace rcl_slam_basic_transform;
    using namespace rcl_pose_graph_type;

    void PoseGraph::addPose(double x, double y, double theta){
        std::lock_guard<std::mutex> lock(mutex_);
        pose_history.push_back(Node{x, y, theta});
    }
    void PoseGraph::addPose(const RobotBasePose& pose){
        std::lock_guard<std::mutex> lock(mutex_);
        pose_history.push_back(Node{pose.tx, pose.ty, pose.theta});
    }
    void PoseGraph::addEdge(int from, int to, double info_tx, double info_ty, double info_theta, bool is_loop){
        std::lock_guard<std::mutex> lock(mutex_);
        if(from < 0 || from >= static_cast<int>(pose_history.size()) || to < 0 || to >= static_cast<int>(pose_history.size()))
            return;

        Edge edge(from, to, info_tx, info_ty, info_theta, is_loop);
        edge.set_relative_pose(Node{relativePose(pose_history[from], pose_history[to])});
        edges.push_back(edge);
    }
    void PoseGraph::addEdge(const rcl_pose_graph_type::Edge& edge){
        std::lock_guard<std::mutex> lock(mutex_);
        edges.push_back(edge);
    }
    size_t PoseGraph::getPoseCount() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_history.size();
    }
    RobotBasePose PoseGraph::getPose(int index) const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(index < 0 || index >= static_cast<int>(pose_history.size()))
            return RobotBasePose();
        return pose_history[index];
    }
    void PoseGraph::setPose(int index, const RobotBasePose& pose){
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= 0 && index < static_cast<int>(pose_history.size())){
            pose_history[index].tx = pose.tx;
            pose_history[index].ty = pose.ty;
            pose_history[index].theta = pose.theta;
        }
    }

    std::vector<Node> PoseGraph::getPoseSnapshot() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_history;
    }

    std::vector<Node> PoseGraph::getPoseSnapshot(int from, int to) const{
        std::lock_guard<std::mutex> lock(mutex_);
        int sz = static_cast<int>(pose_history.size());
        if(from < 0) from = 0;
        if(to > sz) to = sz;
        if(from >= to) return {};
        return std::vector<Node>(pose_history.begin() + from, pose_history.begin() + to);
    }

    void PoseGraph::setPoses(int from, const std::vector<RobotBasePose>& poses){
        std::lock_guard<std::mutex> lock(mutex_);
        int sz = static_cast<int>(pose_history.size());
        for(int i = 0; i < static_cast<int>(poses.size()) && (from + i) < sz; i++){
            pose_history[from + i].tx = poses[i].tx;
            pose_history[from + i].ty = poses[i].ty;
            pose_history[from + i].theta = poses[i].theta;
        }
    }

    // Pose graph optimization 관련 함수들
    Eigen::Vector3d PoseGraph::errorComputeUnlocked(const Edge& edge) const{
        RobotBasePose relative = relativePose(pose_history[edge.from], pose_history[edge.to]);

        Eigen::Vector3d predicted(relative.tx, relative.ty, relative.theta);
        Eigen::Vector3d measurement(edge.relative_pose.tx, edge.relative_pose.ty, edge.relative_pose.theta);
        Eigen::Vector3d error = predicted - measurement;
        error(2) = normalizeAngle(error(2));
        return error;
    }
    Eigen::Vector3d PoseGraph::errorCompute(const Edge& edge) const{
        std::lock_guard<std::mutex> lock(mutex_);
        return errorComputeUnlocked(edge);
    }
    void PoseGraph::loopOptimize(int iter, double epsilon){
        std::lock_guard<std::mutex> lock(mutex_);
        int N = pose_history.size();
        if(N <= 1){
            return;
        }

#ifdef TEST_ICP_HAS_G2O
        using LinearSolver = g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>;
        auto linear_solver = std::make_unique<LinearSolver>();
        auto block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(algorithm);
        optimizer.setVerbose(false);

        for(int i = 0; i < N; i++){
            auto* vertex = new g2o::VertexSE2();
            vertex->setId(i);
            vertex->setEstimate(g2o::SE2(pose_history[i].tx, pose_history[i].ty, pose_history[i].theta));
            vertex->setFixed(i == 0);
            optimizer.addVertex(vertex);
        }

        for(const auto& edge : edges){
            auto* constraint = new g2o::EdgeSE2();
            constraint->setVertex(0, optimizer.vertex(edge.from));
            constraint->setVertex(1, optimizer.vertex(edge.to));
            constraint->setMeasurement(g2o::SE2(edge.relative_pose.tx, edge.relative_pose.ty, edge.relative_pose.theta));

            Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
            information(0, 0) = edge.info_tx;
            information(1, 1) = edge.info_ty;
            information(2, 2) = edge.info_theta;
            constraint->setInformation(information);

            if(edge.is_loop){
                auto* kernel = new g2o::RobustKernelHuber();
                kernel->setDelta(1.0);
                constraint->setRobustKernel(kernel);
            }

            optimizer.addEdge(constraint);
        }

        optimizer.initializeOptimization();
        // 포즈가 많아지면 반복 횟수 제한 (N=500 → 최대10회, N=1000 → 최대5회)
        int max_iter = std::max(3, std::min(iter, 5000 / N));
        optimizer.optimize(max_iter);

        for(int i = 0; i < N; i++){
            const auto* vertex = dynamic_cast<const g2o::VertexSE2*>(optimizer.vertex(i));
            if(!vertex){
                continue;
            }

            const g2o::SE2& estimate = vertex->estimate();
            pose_history[i].tx = estimate.translation()[0];
            pose_history[i].ty = estimate.translation()[1];
            pose_history[i].theta = normalizeAngle(estimate.rotation().angle());
        }
        (void)epsilon;
#else
        // built-in solver: 포즈가 많아지면 반복 횟수 제한
        int max_iter = std::max(3, std::min(iter, 5000 / N));
        using Triplet = Eigen::Triplet<double>;
        for(int i=0; i<max_iter; i++){
            std::vector<Triplet> triplets;
            triplets.reserve(edges.size() * 36);  // 각 엣지가 4개 3x3 블록 기여
            Eigen::VectorXd b = Eigen::VectorXd::Zero(3*N);

            for(const auto& edge: edges){
                Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
                Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
                Eigen::Matrix3d Info = Eigen::Matrix3d::Zero();
                Eigen::Vector3d e = errorComputeUnlocked(edge);

                if(pose_history.size() <= static_cast<size_t>(std::max(edge.from, edge.to)))
                    continue;
                double dx = pose_history[edge.from].tx - pose_history[edge.to].tx;
                double dy = pose_history[edge.from].ty - pose_history[edge.to].ty;
                double theta = pose_history[edge.from].theta;

                Info(0,0) = edge.info_tx; Info(1,1) = edge.info_ty; Info(2,2) = edge.info_theta;
                A(0,0) = -std::cos(theta); A(0,1) = -std::sin(theta); A(0,2) = std::sin(theta)*dx - std::cos(theta)*dy;
                A(1,0) =  std::sin(theta); A(1,1) = -std::cos(theta); A(1,2) = std::cos(theta)*dx + std::sin(theta)*dy;
                A(2,0) = 0;                A(2,1) = 0;                A(2,2) = -1;
                B(0,0) =  std::cos(theta); B(0,1) =  std::sin(theta); B(0, 2) = 0;
                B(1,0) = -std::sin(theta); B(1,1) =  std::cos(theta); B(1, 2) = 0;
                B(2,0) = 0;                B(2,1) = 0;                B(2, 2) = 1;

                int fi = edge.from * 3;
                int ti = edge.to * 3;

                Eigen::Matrix3d AtIA = A.transpose() * Info * A;
                Eigen::Matrix3d BtIB = B.transpose() * Info * B;
                Eigen::Matrix3d AtIB = A.transpose() * Info * B;
                Eigen::Matrix3d BtIA = B.transpose() * Info * A;

                for(int r=0; r<3; r++) for(int c=0; c<3; c++){
                    triplets.emplace_back(fi+r, fi+c, AtIA(r,c));
                    triplets.emplace_back(ti+r, ti+c, BtIB(r,c));
                    triplets.emplace_back(fi+r, ti+c, AtIB(r,c));
                    triplets.emplace_back(ti+r, fi+c, BtIA(r,c));
                }
                b.segment<3>(fi) += A.transpose() * Info * e;
                b.segment<3>(ti) += B.transpose() * Info * e;
            }

            // Anchor first node to prevent singular system
            for(int k=0; k<3; k++) triplets.emplace_back(k, k, 1e6);

            Eigen::SparseMatrix<double> H(3*N, 3*N);
            H.setFromTriplets(triplets.begin(), triplets.end());

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(H);
            Eigen::VectorXd delta = solver.solve(-b);
            for(int i=0; i<N; i++){
                int idx = i * 3;
                pose_history[i].tx += delta(idx);
                pose_history[i].ty += delta(idx + 1);
                pose_history[i].theta = normalizeAngle(pose_history[i].theta + delta(idx + 2));
            }
            double max_delta = delta.cwiseAbs().maxCoeff();
            if(max_delta < epsilon){
                break;
            }
        }
#endif
    }
}
