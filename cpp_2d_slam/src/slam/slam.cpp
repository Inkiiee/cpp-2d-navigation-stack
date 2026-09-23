#include "slam.h"
#include "slam_basic.h"
#include "ros_publisher_node.hpp"

#include <QDebug>
#include <QMetaObject>

#include <algorithm>
#include <chrono>
#include <utility>
#include <vector>

namespace rcl_slam{
    using namespace rcl_scan_match_backend;
    using namespace rcl_loop_detecter;
    using namespace rcl_painter;
    using namespace rcl_slam_basic_transform;

    namespace{
        template<typename Worker>
        void destroyWorker(Worker*& worker, QThread& thread){
            if(worker){
                if(worker->thread() == QThread::currentThread()){
                    delete worker;
                }else if(thread.isRunning()){
                    Worker* worker_to_delete = worker;
                    const bool invoked = QMetaObject::invokeMethod(
                        worker,
                        [worker_to_delete](){ delete worker_to_delete; },
                        Qt::BlockingQueuedConnection
                    );
                    if(!invoked){
                        thread.quit();
                        thread.wait();
                        delete worker;
                    }
                }else{
                    delete worker;
                }
                worker = nullptr;
            }

            if(thread.isRunning()){
                thread.quit();
                thread.wait();
            }
        }
    }

    class MapRebuildWorker: public QObject{
    public:
        MapRebuildWorker(ScanMatchBackend* backend, std::shared_ptr<RosPublisherNode> ros_pub)
            : backend_{backend}, ros_pub_{std::move(ros_pub)} {}

        void rebuildMap(){
            if(!backend_){
                return;
            }

            auto t0 = std::chrono::steady_clock::now();
            auto world_map = backend_->getWorldMap();
            auto pose_graph = backend_->getPoseGraph();
            auto sub_maps = backend_->getSubMaps();
            auto data_mutex = backend_->getSharedDataMutex();

            std::vector<rcl_pose_graph_type::Node> pose_snapshot;
            std::vector<rcl_map_backend::sub_map> submap_snapshot;
            {
                std::lock_guard<std::mutex> lock(*data_mutex);
                pose_snapshot = pose_graph->getPoseSnapshot();
                const size_t count = std::min(pose_snapshot.size(), sub_maps->size());
                pose_snapshot.resize(count);
                submap_snapshot.assign(sub_maps->begin(), sub_maps->begin() + count);
            }
            auto t1 = std::chrono::steady_clock::now();

            struct TransformedSubmap{
                std::vector<double> x;
                std::vector<double> y;
                double sensor_x = 0.0;
                double sensor_y = 0.0;
            };

            const size_t count = pose_snapshot.size();
            std::vector<TransformedSubmap> transformed(count);
            for(size_t i = 0; i < count; ++i){
                const auto& sm = submap_snapshot[i];
                transformed[i].x.assign(sm.x.begin(), sm.x.end());
                transformed[i].y.assign(sm.y.begin(), sm.y.end());

                rotationAndTranslation(
                    pose_snapshot[i].tx,
                    pose_snapshot[i].ty,
                    pose_snapshot[i].theta,
                    transformed[i].x,
                    transformed[i].y
                );

                const double ct = std::cos(pose_snapshot[i].theta);
                const double st = std::sin(pose_snapshot[i].theta);
                transformed[i].sensor_x = ct * sm.sensor_x - st * sm.sensor_y + pose_snapshot[i].tx;
                transformed[i].sensor_y = st * sm.sensor_x + ct * sm.sensor_y + pose_snapshot[i].ty;
            }
            auto t2 = std::chrono::steady_clock::now();

            std::vector<double> sensor_xs(count), sensor_ys(count);
            std::vector<std::vector<double>> point_xs(count), point_ys(count);
            for(size_t i = 0; i < count; ++i){
                sensor_xs[i] = transformed[i].sensor_x;
                sensor_ys[i] = transformed[i].sensor_y;
                point_xs[i] = std::move(transformed[i].x);
                point_ys[i] = std::move(transformed[i].y);
            }

            rcl_map_backend::MapBackend temp_map(world_map->getResolution());
            temp_map.rebuildFromSubmaps(sensor_xs, sensor_ys, point_xs, point_ys);
            {
                std::lock_guard<std::mutex> lock(*data_mutex);
                world_map->swapMapData(temp_map);
            }
            auto t3 = std::chrono::steady_clock::now();

            const auto snapshot_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            const auto transform_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            const auto map_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
            const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t0).count();
            qDebug() << "[TIMING rebuildMap] submaps=" << count
                     << " snapshot=" << snapshot_us << "us transform=" << transform_us
                     << "us mapUpdate=" << map_us << "us total=" << total_us << "us ("
                     << total_us / 1000 << "ms)";

            if(ros_pub_){
                std::vector<int8_t> grid_data;
                int width = 0;
                int height = 0;
                double origin_x = 0.0;
                double origin_y = 0.0;
                {
                    std::lock_guard<std::mutex> lock(*data_mutex);
                    world_map->getOccupancyGridData(grid_data, width, height, origin_x, origin_y);
                }
                if(width > 0 && height > 0){
                    ros_pub_->publishMap(
                        grid_data,
                        width,
                        height,
                        origin_x,
                        origin_y,
                        world_map->getResolution()
                    );
                }
            }
        }

    private:
        ScanMatchBackend* backend_;
        std::shared_ptr<RosPublisherNode> ros_pub_;
    };

    SlamSystem::SlamSystem(
        Bridge* bridge,
        std::shared_ptr<RosPublisherNode> ros_pub,
        double resolution,
        QObject* parent
    ):
        QObject(parent),
        pos_r{resolution},
        backend{new ScanMatchBackend(bridge, resolution)},
        loop_detecter{nullptr},
        painter{nullptr},
        map_rebuilder{nullptr},
        bridge{bridge},
        ros_pub_{std::move(ros_pub)}
    {
        qRegisterMetaType<Eigen::Matrix3d>("Eigen::Matrix3d");
        qRegisterMetaType<size_t>("size_t");

        if(ros_pub_){
            backend->setRosPublisher(ros_pub_);
        }

        loop_detecter = new LoopDetecter(
            backend->getPoseGraph(),
            backend->getSubMaps(),
            backend->getSharedDataMutex()
        );
        map_rebuilder = new MapRebuildWorker(backend, ros_pub_);
        painter = new Painter(
            backend->getPoseGraph(),
            backend->getWorldMap(),
            backend->getLocalMap(),
            pos_r,
            backend->getSharedDataMutex()
        );
        painter->setRosPublisher(ros_pub_);

        QObject::connect(
            loop_detecter,
            &LoopDetecter::optimizedPoseUpdated,
            backend,
            &ScanMatchBackend::poseOptimized,
            Qt::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::subMapUpdated,
            loop_detecter,
            &LoopDetecter::detectLoop,
            Qt::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::predictedPose,
            painter,
            &Painter::predictedPoseUpdate,
            Qt::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::scanUpdated,
            painter,
            &Painter::scanUpdate,
            Qt::QueuedConnection
        );
        QObject::connect(
            bridge,
            &Bridge::planDataReceived,
            painter,
            &Painter::globalPlanUpdate,
            Qt::QueuedConnection
        );
        QObject::connect(
            backend,
            &ScanMatchBackend::rebuildMapRequested,
            map_rebuilder,
            &MapRebuildWorker::rebuildMap,
            Qt::QueuedConnection
        );

        backend_thread_.setObjectName("slam-backend");
        loop_thread_.setObjectName("slam-loop-detection");
        rebuild_thread_.setObjectName("slam-map-rebuild");

        backend->moveToThread(&backend_thread_);
        loop_detecter->moveToThread(&loop_thread_);
        map_rebuilder->moveToThread(&rebuild_thread_);

        backend_thread_.start();
        loop_thread_.start();
        rebuild_thread_.start();

        painter->show();
    }

    SlamSystem::~SlamSystem(){
        if(bridge && backend){
            QObject::disconnect(bridge, nullptr, backend, nullptr);
        }
        if(backend){
            QObject::disconnect(backend, nullptr, nullptr, nullptr);
        }
        if(loop_detecter){
            QObject::disconnect(loop_detecter, nullptr, nullptr, nullptr);
        }

        delete painter;
        painter = nullptr;

        destroyWorker(loop_detecter, loop_thread_);
        destroyWorker(map_rebuilder, rebuild_thread_);
        destroyWorker(backend, backend_thread_);
    }

    void SlamSystem::setSharedMem(SharedMem* shared_mem){
        painter->setSharedMem(shared_mem);
    }

    bool SlamSystem::hasExpectedThreadAffinity() const{
        return backend && loop_detecter && map_rebuilder
            && backend->parent() == nullptr
            && loop_detecter->parent() == nullptr
            && map_rebuilder->parent() == nullptr
            && backend->thread() == &backend_thread_
            && loop_detecter->thread() == &loop_thread_
            && map_rebuilder->thread() == &rebuild_thread_
            && backend->thread() != loop_detecter->thread()
            && backend->thread() != map_rebuilder->thread()
            && loop_detecter->thread() != map_rebuilder->thread()
            && backend_thread_.isRunning()
            && loop_thread_.isRunning()
            && rebuild_thread_.isRunning();
    }
}
