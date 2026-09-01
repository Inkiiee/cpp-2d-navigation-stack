#include "loop_detecter.h"

#include <algorithm>
#include <chrono>
#include <QDebug>

namespace rcl_loop_detecter{
    using namespace rcl_pose_graph;
    using namespace rcl_scan_match;
    using namespace rcl_pose_graph_type;

    namespace{
        constexpr int kLoopDetectionStride = 2;
        constexpr int kMinLoopIndexGap = 12;
        constexpr double kCandidateDistanceThreshold = 1.5;
        constexpr int kMaxCandidatesForRefinement = 3;
        constexpr double kMinInitialAvgScore = 0.12;
        constexpr int kOptimizeEveryNLoopEdges = 2;
    }

    LoopDetecter::LoopDetecter(PoseGraph* pg, std::vector<rcl_map_backend::sub_map>* sm, std::mutex* mutex, QObject* parent)
        : QObject(parent), pose_graph{pg}, sub_maps{sm}, shared_data_mutex{mutex}{}
    LoopDetecter::~LoopDetecter(){}

    Eigen::Matrix3d LoopDetecter::convertPoseToMatrix(const Node& node) const{
        Eigen::Matrix3d mat = Eigen::Matrix3d::Identity();
        double ct = std::cos(node.theta), st = std::sin(node.theta);
        mat(0, 0) = ct; mat(0, 1) = -st; mat(0, 2) = node.tx;
        mat(1, 0) = st; mat(1, 1) = ct;  mat(1, 2) = node.ty;
        return mat;
    }
    Eigen::Matrix3d LoopDetecter::calDeltaTransform(const Node& old_node, const Node& new_node) const{
        Eigen::Matrix3d old_mat = convertPoseToMatrix(old_node);
        Eigen::Matrix3d new_mat = convertPoseToMatrix(new_node);
        return new_mat * old_mat.inverse();
    }

    void LoopDetecter::detectLoop(size_t current_index){
        // 최신 요청 인덱스 갱신
        latest_requested_index_.store(current_index, std::memory_order_relaxed);

        if(!pose_graph || !sub_maps || !shared_data_mutex)
            return;
        if(current_index < static_cast<size_t>(kMinLoopIndexGap)){
            return;
        }
        if(current_index % kLoopDetectionStride != 0){
            return;
        }

        // Stale 체크: 큐에 더 새로운 요청이 있으면 이 요청은 스킵
        if(current_index < latest_requested_index_.load(std::memory_order_relaxed)){
            return;
        }

        std::vector<Candidate> candidates;
        rcl_map_backend::sub_map current_submap;
        Node p;
        {
            std::lock_guard<std::mutex> lock(*shared_data_mutex);
            size_t pose_count = pose_graph->getPoseCount();
            size_t sub_map_count = sub_maps->size();
            if(current_index >= pose_count || current_index >= sub_map_count){
                return;
            }

            // 스냅샷 1회로 전체 포즈 복사 (lock 내부에서 getPose N번 호출 제거)
            auto pose_snapshot = pose_graph->getPoseSnapshot(0, static_cast<int>(current_index) - kMinLoopIndexGap + 1);
            p = pose_graph->getPose(current_index);
            current_submap = (*sub_maps)[current_index];

            for(int i = 0; i < static_cast<int>(pose_snapshot.size()); i++){
                double dx = p.tx - pose_snapshot[i].tx;
                double dy = p.ty - pose_snapshot[i].ty;
                double distance = std::sqrt(dx * dx + dy * dy);

                if(distance < kCandidateDistanceThreshold){
                    candidates.push_back(Candidate{i, distance, (*sub_maps)[i]});
                }
            }
        }

        if(candidates.empty()){
            return;
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs){
            return lhs.distance < rhs.distance;
        });
        if(static_cast<int>(candidates.size()) > kMaxCandidatesForRefinement){
            candidates.resize(kMaxCandidatesForRefinement);
        }

        for(const auto& candidate : candidates){
            const int candidate_index = candidate.index;
            auto candidate_submap = candidate.submap;
            Node temp;
            {
                std::lock_guard<std::mutex> lock(*shared_data_mutex);
                temp = pose_graph->getPose(candidate_index);
            }
            double dx = p.tx - temp.tx;
            double dy = p.ty - temp.ty;

            if(std::sqrt(dx * dx + dy * dy) < 1.5){
                Node init = relativePose(temp, p);

                auto t_lut0 = std::chrono::steady_clock::now();
                LookupTable lut = scan_matcher.buildLookupTable(candidate_submap.x, candidate_submap.y, 0.02, 0.05);
                auto t_lut1 = std::chrono::steady_clock::now();

                double init_score = scan_matcher.scoreCandidate(lut, current_submap.x, current_submap.y, init.tx, init.ty, init.theta);
                double init_avg_score = current_submap.x.empty() ? 0.0 : init_score / current_submap.x.size();
                if(init_avg_score < kMinInitialAvgScore){
                    continue;
                }

                auto t_csm0 = std::chrono::steady_clock::now();
                // CSM: 넓은 범위에서 전수 탐색 → NDT: 정밀 보정
                Param par = scan_matcher.runCSM(current_submap.x, current_submap.y, lut,
                                init.tx, init.ty, init.theta,
                                /*search_xy=*/0.3, /*search_theta=*/0.2,
                                /*coarse_xy_res=*/0.05, /*coarse_angle_res=*/0.02,
                                /*fine_xy_res=*/0.01, /*fine_angle_res=*/0.004);
                auto t_csm1 = std::chrono::steady_clock::now();

                par = scan_matcher.runNDT(current_submap.x, current_submap.y, candidate_submap.x, candidate_submap.y,
                            par.tx, par.ty, par.theta, 0.1, 0.05, 30, 1e-6);
                auto t_ndt1 = std::chrono::steady_clock::now();

                double rmse = scan_matcher.cal_rmse(candidate_submap.x, candidate_submap.y, current_submap.x, current_submap.y, par);

                // CSM score로 검증: 평균 score가 높아야 진짜 루프
                double score = scan_matcher.scoreCandidate(lut, current_submap.x, current_submap.y, par.tx, par.ty, par.theta);
                double avg_score = current_submap.x.empty() ? 0.0 : score / current_submap.x.size();

                auto us_lut = std::chrono::duration_cast<std::chrono::microseconds>(t_lut1 - t_lut0).count();
                auto us_csm = std::chrono::duration_cast<std::chrono::microseconds>(t_csm1 - t_csm0).count();
                auto us_ndt = std::chrono::duration_cast<std::chrono::microseconds>(t_ndt1 - t_csm1).count();
                qDebug() << "[TIMING detectLoop candidate" << candidate_index << "]"
                         << " LUT=" << us_lut << "us CSM=" << us_csm << "us NDT=" << us_ndt << "us"
                         << " refPts=" << candidate_submap.x.size() << " scanPts=" << current_submap.x.size()
                         << " avg_score=" << avg_score << " rmse=" << rmse;

                if(rmse < 0.5 && avg_score > 0.3){
                    bool should_optimize = false;
                    int last_node_index = -1;
                    Node old_node;
                    {
                        std::lock_guard<std::mutex> lock(*shared_data_mutex);

                        Edge e(candidate_index, current_index, 100.0, 100.0, 100.0, true);
                        e.set_relative_pose(par);
                        pose_graph->addEdge(e);

                        pending_loop_edges_++;
                        should_optimize = (pending_loop_edges_ >= kOptimizeEveryNLoopEdges);
                    }

                    if(should_optimize){
                        // loopOptimize를 shared_data_mutex 밖에서 실행
                        // PoseGraph 내부 mutex_만 사용 → lidarUpdate의 다른 구간은 블록되지 않음
                        {
                            std::lock_guard<std::mutex> lock(*shared_data_mutex);
                            last_node_index = pose_graph->getPoseCount() - 1;
                            old_node = pose_graph->getPose(last_node_index);
                        }

                        auto t_opt0 = std::chrono::steady_clock::now();
                        pose_graph->loopOptimize();
                        auto t_opt1 = std::chrono::steady_clock::now();
                        pending_loop_edges_ = 0;

                        Node new_node = pose_graph->getPose(last_node_index);
                        Eigen::Matrix3d delta = calDeltaTransform(old_node, new_node);

                        auto us_opt = std::chrono::duration_cast<std::chrono::microseconds>(t_opt1 - t_opt0).count();
                        qDebug() << "[TIMING loopOptimize] poses=" << pose_graph->getPoseCount()
                                 << " time=" << us_opt << "us (" << us_opt / 1000 << "ms)";

                        emit optimizedPoseUpdated(last_node_index, delta);
                    }

                    qDebug()<<"Loop closure detected between pose "<<current_index<<" and pose "<<candidate_index
                            <<" candidates "<<candidates.size()<<" optimized "<<should_optimize;
                    break;
                }
            }
        }
    }
}
