#include "slam.h"
#include "slam_basic.h"
#include "ros_publisher_node.hpp"

#include <chrono>
#include <QThread>
#include <QDebug>

namespace rcl_slam{
    using namespace rcl_scan_match_backend;
    using namespace rcl_loop_detecter;
    using namespace rcl_painter;
    using namespace rcl_slam_basic_transform;
    using rcl_pose_graph_type::Node;

    SlamSystem::SlamSystem(Bridge* b, std::shared_ptr<RosPublisherNode> ros_pub, double r, QObject* parent): QObject(parent), pos_r{r}, bridge{b}, ros_pub_{ros_pub} {
        backend = new ScanMatchBackend(bridge, pos_r, this);
        if(ros_pub_) backend->setRosPublisher(ros_pub_);
        loop_detecter = new LoopDetecter(backend->getPoseGraph(), backend->getSubMaps(), backend->getSharedDataMutex(), this);
        painter = new Painter(backend->getPoseGraph(), backend->getWorldMap(), backend->getLocalMap(), pos_r, backend->getSharedDataMutex());
        painter->setRosPublisher(ros_pub_);

        QObject::connect(
            loop_detecter, 
            &LoopDetecter::optimizedPoseUpdated, 
            backend,
            &ScanMatchBackend::poseOptimized,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::subMapUpdated,
            loop_detecter, 
            &LoopDetecter::detectLoop,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::predictedPose, 
            painter, 
            &Painter::predictedPoseUpdate,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend, 
            &ScanMatchBackend::scanUpdated,
            painter,
            &Painter::scanUpdate,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            bridge,
            &Bridge::planDataReceived,
            painter,
            &Painter::globalPlanUpdate,
            Qt::ConnectionType::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::rebuildMapRequested,
            this,
            &SlamSystem::rebuildMap,
            Qt::ConnectionType::QueuedConnection
        );

        painter->show();

        QThread * thread1 = new QThread(this);
        backend->moveToThread(thread1);
        thread1->start();

        QThread * thread2 = new QThread(this);
        loop_detecter->moveToThread(thread2);
        thread2->start();

        QThread * thread3 = new QThread(this);
        this->moveToThread(thread3);
        thread3->start();
    }
    SlamSystem::~SlamSystem(){
        backend->deleteLater();
        loop_detecter->deleteLater();
        painter->deleteLater();
    }

    void SlamSystem::rebuildMap(){
        auto t0 = std::chrono::steady_clock::now();
        auto world_map = backend->getWorldMap();
        auto pose_graph = backend->getPoseGraph();
        auto sub_maps = backend->getSubMaps();
        auto data_mutex = backend->getSharedDataMutex();

        // 스냅샷으로 포즈 복사 + 서브맵 참조 준비 (짧은 lock)
        std::vector<rcl_pose_graph_type::Node> pose_snapshot;
        size_t count;
        {
            std::lock_guard<std::mutex> lock(*data_mutex);
            pose_snapshot = pose_graph->getPoseSnapshot();
            count = std::min(pose_snapshot.size(), sub_maps->size());
        }
        auto t1 = std::chrono::steady_clock::now();

        // 변환 연산은 lock 없이 수행
        struct TransformedSubmap {
            std::vector<double> x, y;
            double sensor_x, sensor_y;
        };
        std::vector<TransformedSubmap> transformed(count);

        for(size_t i = 0; i < count; i++){
            const auto& sm = (*sub_maps)[i];
            transformed[i].x.assign(sm.x.begin(), sm.x.end());
            transformed[i].y.assign(sm.y.begin(), sm.y.end());

            rotationAndTranslation(pose_snapshot[i].tx, pose_snapshot[i].ty, pose_snapshot[i].theta,
                                   transformed[i].x, transformed[i].y);

            double sx = sm.sensor_x, sy = sm.sensor_y;
            double ct = std::cos(pose_snapshot[i].theta), st = std::sin(pose_snapshot[i].theta);
            transformed[i].sensor_x = ct * sx - st * sy + pose_snapshot[i].tx;
            transformed[i].sensor_y = st * sx + ct * sy + pose_snapshot[i].ty;
        }
        auto t2 = std::chrono::steady_clock::now();

        // 맵을 lock 없이 임시 객체에 빌드, 짧은 lock으로 swap
        // flat 2D array 기반 Bresenham → hash map 20.6M ops を O(1) 배열 접근으로
        std::vector<double> sensor_xs(count), sensor_ys(count);
        std::vector<std::vector<double>> pt_xs(count), pt_ys(count);
        for(size_t i = 0; i < count; i++){
            sensor_xs[i] = transformed[i].sensor_x;
            sensor_ys[i] = transformed[i].sensor_y;
            pt_xs[i] = std::move(transformed[i].x);
            pt_ys[i] = std::move(transformed[i].y);
        }

        rcl_map_backend::MapBackend temp_map(world_map->getResolution());
        temp_map.rebuildFromSubmaps(sensor_xs, sensor_ys, pt_xs, pt_ys);
        {
            std::lock_guard<std::mutex> lock(*data_mutex);
            world_map->swapMapData(temp_map);
        }
        auto t3 = std::chrono::steady_clock::now();

        auto us_snap = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        auto us_xform = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto us_map = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        auto us_total = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t0).count();
        qDebug() << "[TIMING rebuildMap] submaps=" << count
                 << " snapshot=" << us_snap << "us transform=" << us_xform
                 << "us mapUpdate=" << us_map << "us total=" << us_total << "us (" << us_total / 1000 << "ms)";

        // ROS2 맵 퍼블리시 (throttle은 내부에서 처리)
        if(ros_pub_){
            std::vector<int8_t> grid_data;
            int gw = 0, gh = 0;
            double ox = 0, oy = 0;
            {
                std::lock_guard<std::mutex> lock(*data_mutex);
                world_map->getOccupancyGridData(grid_data, gw, gh, ox, oy);
            }
            if(gw > 0 && gh > 0){
                ros_pub_->publishMap(grid_data, gw, gh, ox, oy, world_map->getResolution());
            }
        }
    }

    void SlamSystem::setSharedMem(SharedMem* sm){
        painter->setSharedMem(sm);
    }
}
