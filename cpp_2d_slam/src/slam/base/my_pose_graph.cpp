#include "my_pose_graph.h"
#include "slam_basic.h"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#ifdef CPP_2D_SLAM_HAS_G2O
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/types_slam2d.h>
#endif

namespace rcl_pose_graph_type{
    using rcl_slam_basic_type::RobotBasePose;

    Node::Node(double x, double y, double theta): RobotBasePose(x, y, theta) {}
    Node::Node(const Node& node): RobotBasePose(node.tx, node.ty, node.theta) {}
    Node::Node(const RobotBasePose& pose): RobotBasePose(pose.tx, pose.ty, pose.theta) {}

    Edge::Edge(int from_index, int to_index, double tx_info, double ty_info, double theta_info, bool loop):
        from(from_index),
        to(to_index),
        info_tx(tx_info),
        info_ty(ty_info),
        info_theta(theta_info),
        is_loop(loop) {}

    void Edge::set_relative_pose(const Node& node){
        relative_pose.tx = node.tx;
        relative_pose.ty = node.ty;
        relative_pose.theta = node.theta;
    }
}

namespace{
    using rcl_pose_graph_type::Edge;
    using rcl_pose_graph_type::Node;

    bool isFinitePose(const Node& pose){
        return std::isfinite(pose.tx)
            && std::isfinite(pose.ty)
            && std::isfinite(pose.theta);
    }

    bool hasValidInformation(const Edge& edge){
        return std::isfinite(edge.info_tx)
            && std::isfinite(edge.info_ty)
            && std::isfinite(edge.info_theta)
            && edge.info_tx > 0.0
            && edge.info_ty > 0.0
            && edge.info_theta > 0.0;
    }

    bool isValidEdge(const Edge& edge, std::size_t pose_count){
        return edge.from >= 0
            && edge.to >= 0
            && static_cast<std::size_t>(edge.from) < pose_count
            && static_cast<std::size_t>(edge.to) < pose_count
            && isFinitePose(edge.relative_pose)
            && hasValidInformation(edge);
    }
}

namespace rcl_pose_graph{
    using namespace rcl_slam_basic_type;
    using namespace rcl_slam_basic_transform;
    using namespace rcl_pose_graph_type;

    void PoseGraph::addPose(double x, double y, double theta){
        addPose(RobotBasePose{x, y, theta});
    }

    void PoseGraph::addPose(const RobotBasePose& pose){
        Node node{pose};
        if(!isFinitePose(node)){
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        pose_history.push_back(node);
    }

    void PoseGraph::addEdge(int from, int to, double info_tx, double info_ty, double info_theta, bool is_loop){
        std::lock_guard<std::mutex> lock(mutex_);
        Edge edge(from, to, info_tx, info_ty, info_theta, is_loop);
        if(from >= 0
            && to >= 0
            && static_cast<std::size_t>(from) < pose_history.size()
            && static_cast<std::size_t>(to) < pose_history.size()){
            edge.set_relative_pose(Node{relativePose(pose_history[from], pose_history[to])});
        }
        if(isValidEdge(edge, pose_history.size())){
            edges.push_back(edge);
        }
    }

    void PoseGraph::addEdge(const Edge& edge){
        std::lock_guard<std::mutex> lock(mutex_);
        if(isValidEdge(edge, pose_history.size())){
            edges.push_back(edge);
        }
    }

    size_t PoseGraph::getPoseCount() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_history.size();
    }

    RobotBasePose PoseGraph::getPose(int index) const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(index < 0 || index >= static_cast<int>(pose_history.size())){
            return RobotBasePose();
        }
        return pose_history[index];
    }

    void PoseGraph::setPose(int index, const RobotBasePose& pose){
        Node node{pose};
        if(!isFinitePose(node)){
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if(index >= 0 && index < static_cast<int>(pose_history.size())){
            pose_history[index] = node;
        }
    }

    std::vector<Node> PoseGraph::getPoseSnapshot() const{
        std::lock_guard<std::mutex> lock(mutex_);
        return pose_history;
    }

    std::vector<Node> PoseGraph::getPoseSnapshot(int from, int to) const{
        std::lock_guard<std::mutex> lock(mutex_);
        const int size = static_cast<int>(pose_history.size());
        from = std::max(from, 0);
        to = std::min(to, size);
        if(from >= to){
            return {};
        }
        return std::vector<Node>(pose_history.begin() + from, pose_history.begin() + to);
    }

    void PoseGraph::setPoses(int from, const std::vector<RobotBasePose>& poses){
        if(from < 0){
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        const int size = static_cast<int>(pose_history.size());
        for(int i = 0; i < static_cast<int>(poses.size()) && from + i < size; ++i){
            Node node{poses[i]};
            if(isFinitePose(node)){
                pose_history[from + i] = node;
            }
        }
    }

    Eigen::Vector3d PoseGraph::errorComputeUnlocked(const Edge& edge) const{
        RobotBasePose relative = relativePose(pose_history[edge.from], pose_history[edge.to]);
        Eigen::Vector3d predicted(relative.tx, relative.ty, relative.theta);
        Eigen::Vector3d measurement(
            edge.relative_pose.tx,
            edge.relative_pose.ty,
            edge.relative_pose.theta
        );
        Eigen::Vector3d error = predicted - measurement;
        error(2) = normalizeAngle(error(2));
        return error;
    }

    Eigen::Vector3d PoseGraph::errorCompute(const Edge& edge) const{
        std::lock_guard<std::mutex> lock(mutex_);
        if(!isValidEdge(edge, pose_history.size())){
            return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        }
        return errorComputeUnlocked(edge);
    }

    bool PoseGraph::loopOptimize(int iter, double epsilon){
        std::lock_guard<std::mutex> lock(mutex_);
        const int pose_count = static_cast<int>(pose_history.size());
        if(pose_count <= 1){
            return true;
        }
        if(iter <= 0 || !std::isfinite(epsilon) || epsilon <= 0.0 || edges.empty()){
            return false;
        }
        for(const auto& pose : pose_history){
            if(!isFinitePose(pose)){
                return false;
            }
        }
        for(const auto& edge : edges){
            if(!isValidEdge(edge, pose_history.size())){
                return false;
            }
        }

        const int max_iter = std::max(3, std::min(iter, 5000 / pose_count));

#ifdef CPP_2D_SLAM_HAS_G2O
        using LinearSolver = g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>;
        auto linear_solver = std::make_unique<LinearSolver>();
        auto block_solver = std::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
        auto* algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(algorithm);
        optimizer.setVerbose(false);

        for(int i = 0; i < pose_count; ++i){
            auto* vertex = new g2o::VertexSE2();
            vertex->setId(i);
            vertex->setEstimate(g2o::SE2(pose_history[i].tx, pose_history[i].ty, pose_history[i].theta));
            vertex->setFixed(i == 0);
            if(!optimizer.addVertex(vertex)){
                delete vertex;
                return false;
            }
        }

        for(const auto& edge : edges){
            auto* constraint = new g2o::EdgeSE2();
            constraint->setVertex(0, optimizer.vertex(edge.from));
            constraint->setVertex(1, optimizer.vertex(edge.to));
            constraint->setMeasurement(g2o::SE2(
                edge.relative_pose.tx,
                edge.relative_pose.ty,
                edge.relative_pose.theta
            ));

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
            if(!optimizer.addEdge(constraint)){
                delete constraint;
                return false;
            }
        }

        if(!optimizer.initializeOptimization()){
            return false;
        }
        if(optimizer.optimize(max_iter) < 0){
            return false;
        }

        std::vector<Node> optimized_poses = pose_history;
        for(int i = 0; i < pose_count; ++i){
            const auto* vertex = dynamic_cast<const g2o::VertexSE2*>(optimizer.vertex(i));
            if(!vertex){
                return false;
            }

            const g2o::SE2& estimate = vertex->estimate();
            Node optimized{
                estimate.translation()[0],
                estimate.translation()[1],
                normalizeAngle(estimate.rotation().angle())
            };
            if(!isFinitePose(optimized)){
                return false;
            }
            optimized_poses[i] = optimized;
        }
        pose_history = std::move(optimized_poses);
        return true;
#else
        const std::vector<Node> original_poses = pose_history;
        using Triplet = Eigen::Triplet<double>;

        for(int iteration = 0; iteration < max_iter; ++iteration){
            std::vector<Triplet> triplets;
            triplets.reserve(edges.size() * 36 + 3);
            Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * pose_count);

            for(const auto& edge : edges){
                Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
                Eigen::Matrix3d B = Eigen::Matrix3d::Zero();
                Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
                const Eigen::Vector3d error = errorComputeUnlocked(edge);

                const double dx = pose_history[edge.from].tx - pose_history[edge.to].tx;
                const double dy = pose_history[edge.from].ty - pose_history[edge.to].ty;
                const double theta = pose_history[edge.from].theta;

                information(0, 0) = edge.info_tx;
                information(1, 1) = edge.info_ty;
                information(2, 2) = edge.info_theta;
                A(0, 0) = -std::cos(theta);
                A(0, 1) = -std::sin(theta);
                A(0, 2) = std::sin(theta) * dx - std::cos(theta) * dy;
                A(1, 0) = std::sin(theta);
                A(1, 1) = -std::cos(theta);
                A(1, 2) = std::cos(theta) * dx + std::sin(theta) * dy;
                A(2, 2) = -1.0;
                B(0, 0) = std::cos(theta);
                B(0, 1) = std::sin(theta);
                B(1, 0) = -std::sin(theta);
                B(1, 1) = std::cos(theta);
                B(2, 2) = 1.0;

                const int from_index = edge.from * 3;
                const int to_index = edge.to * 3;
                const Eigen::Matrix3d AtIA = A.transpose() * information * A;
                const Eigen::Matrix3d BtIB = B.transpose() * information * B;
                const Eigen::Matrix3d AtIB = A.transpose() * information * B;
                const Eigen::Matrix3d BtIA = B.transpose() * information * A;
                if(!error.allFinite()
                    || !AtIA.allFinite()
                    || !BtIB.allFinite()
                    || !AtIB.allFinite()
                    || !BtIA.allFinite()){
                    pose_history = original_poses;
                    return false;
                }

                for(int row = 0; row < 3; ++row){
                    for(int column = 0; column < 3; ++column){
                        triplets.emplace_back(from_index + row, from_index + column, AtIA(row, column));
                        triplets.emplace_back(to_index + row, to_index + column, BtIB(row, column));
                        triplets.emplace_back(from_index + row, to_index + column, AtIB(row, column));
                        triplets.emplace_back(to_index + row, from_index + column, BtIA(row, column));
                    }
                }
                b.segment<3>(from_index) += A.transpose() * information * error;
                b.segment<3>(to_index) += B.transpose() * information * error;
            }

            for(int axis = 0; axis < 3; ++axis){
                triplets.emplace_back(axis, axis, 1e6);
            }

            Eigen::SparseMatrix<double> H(3 * pose_count, 3 * pose_count);
            H.setFromTriplets(triplets.begin(), triplets.end());
            H.makeCompressed();

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(H);
            if(solver.info() != Eigen::Success){
                pose_history = original_poses;
                return false;
            }

            const Eigen::VectorXd delta = solver.solve(-b);
            if(solver.info() != Eigen::Success || !delta.allFinite()){
                pose_history = original_poses;
                return false;
            }

            for(int i = 0; i < pose_count; ++i){
                const int index = i * 3;
                Node updated{
                    pose_history[i].tx + delta(index),
                    pose_history[i].ty + delta(index + 1),
                    normalizeAngle(pose_history[i].theta + delta(index + 2))
                };
                if(!isFinitePose(updated)){
                    pose_history = original_poses;
                    return false;
                }
                pose_history[i] = updated;
            }

            const double max_delta = delta.cwiseAbs().maxCoeff();
            if(!std::isfinite(max_delta)){
                pose_history = original_poses;
                return false;
            }
            if(max_delta < epsilon){
                break;
            }
        }
        return true;
#endif
    }
}
