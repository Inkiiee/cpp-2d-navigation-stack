#include "scan_match_backend.h"
#include "slam_basic.h"
#include "my_pose_graph.h"
#include "ros_publisher_node.hpp"

#include <cmath>
#include <chrono>
#include <QDebug>

namespace rcl_scan_match_backend{
    using namespace rcl_scan_match;
    using namespace rcl_slam_basic_type;
    using namespace rcl_slam_basic_transform;
    using namespace rcl_pose_graph;
    using namespace rcl_map_backend;

    ScanMatchBackend::ScanMatchBackend(Bridge* b, double pos_r, QObject* parent): QObject(parent), bridge{b}, world_map(pos_r), local_map(pos_r){
        map_x = 0, map_y = 0, map_theta = 0;
        odom_x = 0, odom_y = 0, odom_theta = 0;
        imu_theta = 0;

        // lidarUpdate only copies into the mailbox. Running it in the producer
        // thread prevents a payload-bearing event from accumulating per scan.
        QObject::connect(bridge, &Bridge::scanDataReceived, this, &ScanMatchBackend::lidarUpdate, Qt::ConnectionType::DirectConnection);
        QObject::connect(bridge, &Bridge::odomDataReceived, this, &ScanMatchBackend::odomUpdate, Qt::ConnectionType::QueuedConnection);
        // QObject::connect(bridge, &Bridge::imuHeadingReceived, this, &ScanMatchBackend::imuUpdate, Qt::ConnectionType::QueuedConnection);
    }
    ScanMatchBackend::~ScanMatchBackend(){}

    void ScanMatchBackend::setRosPublisher(std::shared_ptr<RosPublisherNode> pub){
        ros_pub_ = pub;
    }

    std::vector<sub_map>* ScanMatchBackend::getSubMaps(){
        return &sub_maps;
    }
    PoseGraph* ScanMatchBackend::getPoseGraph(){
        return &pose_graph;
    }
    MapBackend* ScanMatchBackend::getWorldMap(){
        return &world_map;
    }
    MapBackend* ScanMatchBackend::getLocalMap(){
        return &local_map;
    }
    std::mutex* ScanMatchBackend::getSharedDataMutex(){
        return &shared_data_mutex_;
    }

    void ScanMatchBackend::odomUpdate(double x, double y, double /*z*/, double /*rx*/, double /*ry*/, double rz){
        double change_x = x - odom_x;
        double change_y = y - odom_y;
        double heading = rz;
        double change_theta = normalizeAngle(heading - odom_theta);

        double local_dx = cos(odom_theta) * change_x + sin(odom_theta) * change_y;
        double local_dy = -sin(odom_theta) * change_x + cos(odom_theta) * change_y;
        map_x = map_x + (cos(map_theta) * local_dx - sin(map_theta) * local_dy);
        map_y = map_y + (sin(map_theta) * local_dx + cos(map_theta) * local_dy);
        map_theta = normalizeAngle(map_theta + change_theta);

        odom_x = x;
        odom_y = y;
        odom_theta = heading;
    }

    void ScanMatchBackend::imuUpdate(double yaw){
        imu_theta = normalizeAngle(yaw);
    }

    void ScanMatchBackend::poseOptimized(int index, const Eigen::Matrix3d& delta){
        double delta_theta = std::atan2(delta(1, 0), delta(0, 0));

        // 1) 스냅샷 읽기 (짧은 lock)
        auto snapshot = pose_graph.getPoseSnapshot(index + 1, static_cast<int>(pose_graph.getPoseCount()));

        // 2) delta 적용 (lock 없이)
        std::vector<RobotBasePose> updated;
        updated.reserve(snapshot.size());
        for(const auto& p : snapshot){
            Eigen::Vector3d vec(p.tx, p.ty, 1);
            vec = delta * vec;
            updated.push_back(RobotBasePose(vec.x(), vec.y(), normalizeAngle(p.theta + delta_theta)));
        }

        // 3) 일괄 쓰기 + map 보정 (짧은 lock)
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex_);
            pose_graph.setPoses(index + 1, updated);

            Eigen::Vector3d vec(map_x, map_y, 1);
            vec = delta * vec;
            map_x = vec.x();
            map_y = vec.y();
            map_theta = normalizeAngle(map_theta + delta_theta);
        }

        ref_cache_valid_ = false;  // rebuildMap이 world_map을 교체하므로
        force_csm_ = true;
        emit rebuildMapRequested();
    }

    double preciouse(double val, int num){
        double temp = (std::pow(10, num));
        return std::round(temp * val) / temp;
    }

    void ScanMatchBackend::lidarUpdate(const ScanAxis& xs, const ScanAxis& ys){
        if(scan_mailbox_.submit(xs, ys)){
            QMetaObject::invokeMethod(this, [this](){ processLatestScan(); }, Qt::QueuedConnection);
        }
    }

    void ScanMatchBackend::processLatestScan(){
        ScanAxis latest_xs;
        ScanAxis latest_ys;
        if(!scan_mailbox_.takeLatest(latest_xs, latest_ys)){
            if(scan_mailbox_.completeProcessing()){
                QMetaObject::invokeMethod(this, [this](){ processLatestScan(); }, Qt::QueuedConnection);
            }
            return;
        }

        processScan(latest_xs, latest_ys);

        // Process one scan per Qt event so other backend events are not starved.
        // If producers overwrote the mailbox meanwhile, reserve one follow-up.
        if(scan_mailbox_.completeProcessing()){
            QMetaObject::invokeMethod(this, [this](){ processLatestScan(); }, Qt::QueuedConnection);
        }
    }

    void ScanMatchBackend::processScan(const ScanAxis& latest_xs, const ScanAxis& latest_ys){
        // ── 거리 게이팅 ──
        auto t_start = std::chrono::steady_clock::now();

        double travel_xy = std::sqrt((map_x - last_match_x_) * (map_x - last_match_x_) + (map_y - last_match_y_) * (map_y - last_match_y_));
        double travel_theta = std::abs(normalizeAngle(map_theta - last_match_theta_));
        bool moved_enough = (travel_xy >= kMinTravelDistance || travel_theta >= kMinTravelAngle);

        if(!moved_enough){
            std::vector<double> pixel_x(latest_xs.begin(), latest_xs.end()), pixel_y(latest_ys.begin(), latest_ys.end());
            rotationAndTranslation(map_x, map_y, map_theta, pixel_x, pixel_y);
            local_map.updateOccupancyMap(map_x, map_y, pixel_x, pixel_y);
            emit scanUpdated(pixel_x, pixel_y);
            emit predictedPose(map_x, map_y, map_theta);
            if(ros_pub_) ros_pub_->publishPoseAndTF(map_x, map_y, map_theta, odom_x, odom_y, odom_theta);
            return;
        }

        if(frame_index++ % 5 == 0){
            rcl_map_backend_type::sub_map sm;
            local_map.getPos(sm.x, sm.y, true);  // static_only: hit 비율 높은 셀만

            RobotBasePose currentPose{map_x, map_y, map_theta};
            // 로컬 좌표계에서의 센서 원점 (로컬 기준이므로 0,0)
            sm.sensor_x = 0;
            sm.sensor_y = 0;
            int current_index = 0;
            {
                std::lock_guard<std::mutex> lock(shared_data_mutex_);

                // 서브맵 포인트는 이미 월드 좌표 → addWorldMap에 센서 위치도 전달
                world_map.updateOccupancyMap(map_x, map_y, sm.x, sm.y);

                // Local 좌표계로 서브맵으로 저장시.
                RobotBasePose inv = inversePose(currentPose);
                rotationAndTranslation(inv.tx, inv.ty, inv.theta, sm.x, sm.y);
                sub_maps.push_back(sm);

                current_index = pose_graph.getPoseCount();
                pose_graph.addPose(currentPose);

                // 연속 노드 간 오도메트리 엣지 추가 (루프 최적화 시 궤적 유지에 필수)
                if(current_index > 0){
                    pose_graph.addEdge(current_index - 1, current_index, 1.0, 1.0, 1.0, false);
                }

                local_map.clearMap();
            }

            frame_index = 1;
            force_csm_ = true;  // reference map이 바뀌었으니 다음 매칭에서 coarse search
            ref_cache_valid_ = false;  // world_map이 바뀌었으니 ref 캐시도 무효화
            qDebug()<<"Sub "<<preciouse(map_x, 3)<<" "<<preciouse(map_y, 3)<<" "<<preciouse(map_theta, 3);
            qDebug()<<"Odom "<<preciouse(odom_x, 3)<<" "<<preciouse(odom_y, 3)<<" "<<preciouse(odom_theta, 3);

            emit subMapUpdated(current_index);

            // 서브맵 생성 시 ROS2 맵 퍼블리시 (내부 throttle 적용)
            if(ros_pub_){
                std::vector<int8_t> grid_data;
                int gw = 0, gh = 0;
                double ox = 0, oy = 0;
                {
                    std::lock_guard<std::mutex> lock(shared_data_mutex_);
                    world_map.getOccupancyGridData(grid_data, gw, gh, ox, oy);
                }
                if(gw > 0 && gh > 0){
                    ros_pub_->publishMap(grid_data, gw, gh, ox, oy, world_map.getResolution());
                }
            }
        }

        // 매칭 대상: local_map + world_map에서 로봇 근처 다운샘플된 포인트
        auto t_submap = std::chrono::steady_clock::now();
        match_ref_map_.clearMap();
        std::vector<double> pixel_x, pixel_y, world_x, world_y;
        local_map.getPos(world_x, world_y, true);
        match_ref_map_.addPos(world_x, world_y);

        // world_map ref 캐시: 맵 변경 또는 0.5m 이상 이동 시에만 재계산
        double cache_dist = std::sqrt((map_x - ref_cache_cx_) * (map_x - ref_cache_cx_) + (map_y - ref_cache_cy_) * (map_y - ref_cache_cy_));
        if(!ref_cache_valid_ || cache_dist > 0.5){
            std::lock_guard<std::mutex> lock(shared_data_mutex_);
            world_map.getAdjacentPosDownsampled(RobotBasePose(map_x, map_y, map_theta), cached_world_x_, cached_world_y_, 5.0, 0.1, true);
            ref_cache_cx_ = map_x;
            ref_cache_cy_ = map_y;
            ref_cache_valid_ = true;
        }
        match_ref_map_.addPos(cached_world_x_, cached_world_y_);
        match_ref_map_.getPos(world_x, world_y);

        auto t_ref = std::chrono::steady_clock::now();
        int ref_pts = static_cast<int>(world_x.size());

        if(!world_x.empty()){
            std::vector<double> scan_x(latest_xs.begin(), latest_xs.end()), scan_y(latest_ys.begin(), latest_ys.end());
            int scan_pts = static_cast<int>(scan_x.size());

            // odom 변위가 충분할 때만 CSM, 그 외엔 NDT만
            double disp_xy = std::sqrt((map_x - last_csm_x_) * (map_x - last_csm_x_) + (map_y - last_csm_y_) * (map_y - last_csm_y_));
            double disp_theta = std::abs(normalizeAngle(map_theta - last_csm_theta_));
            bool need_csm = (disp_xy > 0.15 || disp_theta > 0.1 || force_csm_);

            Param p;
            if(need_csm){
                // LUT는 현재 reference에 종속되므로 CSM을 실행하는 이 프레임에서만 사용한다.
                auto t_lut0 = std::chrono::steady_clock::now();
                LookupTable lut = scan_matcher.buildLookupTable(world_x, world_y, 0.02, 0.05);
                auto t_lut1 = std::chrono::steady_clock::now();

                p = scan_matcher.runCSM(scan_x, scan_y, lut, map_x, map_y, map_theta,
                        0.3, 0.2, 0.05, 0.02, 0.005, 0.002);
                auto t_csm1 = std::chrono::steady_clock::now();

                p = scan_matcher.runNDT(scan_x, scan_y, world_x, world_y, p.tx, p.ty, p.theta, 0.1, 0.05, 30, 1e-6);
                auto t_ndt1 = std::chrono::steady_clock::now();

                last_csm_x_ = p.tx;
                last_csm_y_ = p.ty;
                last_csm_theta_ = p.theta;
                force_csm_ = false;

                auto ms_lut = std::chrono::duration_cast<std::chrono::microseconds>(t_lut1 - t_lut0).count();
                auto ms_csm = std::chrono::duration_cast<std::chrono::microseconds>(t_csm1 - t_lut1).count();
                auto ms_ndt = std::chrono::duration_cast<std::chrono::microseconds>(t_ndt1 - t_csm1).count();
                qDebug() << "[TIMING lidarUpdate CSM] ref=" << ref_pts << " scan=" << scan_pts
                         << " LUT=" << ms_lut << "us CSM=" << ms_csm << "us NDT=" << ms_ndt << "us";
            } else {
                // NDT만 (odom이 좋은 초기값)
                auto t_ndt0 = std::chrono::steady_clock::now();
                p = scan_matcher.runNDT(scan_x, scan_y, world_x, world_y, map_x, map_y, map_theta, 0.1, 0.05, 30, 1e-6);
                auto t_ndt1 = std::chrono::steady_clock::now();
                auto ms_ndt = std::chrono::duration_cast<std::chrono::microseconds>(t_ndt1 - t_ndt0).count();
                qDebug() << "[TIMING lidarUpdate NDT-only] ref=" << ref_pts << " scan=" << scan_pts
                         << " NDT=" << ms_ndt << "us";
            }
            p.theta = normalizeAngle(p.theta);

            // 매칭 결과가 odom 예측과 너무 다르면 가중 평균 (급격한 점프 방지)
            double jump_xy = std::sqrt((p.tx - map_x) * (p.tx - map_x) + (p.ty - map_y) * (p.ty - map_y));
            double jump_theta = std::abs(normalizeAngle(p.theta - map_theta));
            if(jump_xy > 0.3 || jump_theta > 0.15){
                constexpr double alpha = 0.3;
                p.tx = map_x + alpha * (p.tx - map_x);
                p.ty = map_y + alpha * (p.ty - map_y);
                p.theta = normalizeAngle(map_theta + alpha * normalizeAngle(p.theta - map_theta));
            }

            map_x = p.tx;
            map_y = p.ty;
            map_theta = p.theta;
        }

        pixel_x.assign(latest_xs.begin(), latest_xs.end());
        pixel_y.assign(latest_ys.begin(), latest_ys.end());
        rotationAndTranslation(map_x, map_y, map_theta, pixel_x, pixel_y);

        local_map.updateOccupancyMap(map_x, map_y, pixel_x, pixel_y);

        last_match_x_ = map_x;
        last_match_y_ = map_y;
        last_match_theta_ = map_theta;

        auto t_end = std::chrono::steady_clock::now();
        auto ms_submap = std::chrono::duration_cast<std::chrono::microseconds>(t_submap - t_start).count();
        auto ms_ref = std::chrono::duration_cast<std::chrono::microseconds>(t_ref - t_submap).count();
        auto ms_total = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
        qDebug() << "[TIMING lidarUpdate TOTAL] submap=" << ms_submap << "us refBuild=" << ms_ref
                 << "us total=" << ms_total << "us";

        emit scanUpdated(pixel_x, pixel_y);
        emit predictedPose(map_x, map_y, map_theta);
        if(ros_pub_) ros_pub_->publishPoseAndTF(map_x, map_y, map_theta, odom_x, odom_y, odom_theta);

    }
}
