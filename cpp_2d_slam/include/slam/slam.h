#ifndef __RCL_SLAM_H__
#define __RCL_SLAM_H__

#include "scan_match_backend.h"
#include "loop_detecter.h"
#include "bridge.h"
#include "painter.h"

#include <QObject>
#include <QThread>
#include <memory>

class RosPublisherNode;

class SharedMem;

namespace rcl_slam{
    class MapRebuildWorker;

    class SlamSystem: public QObject{
        Q_OBJECT
    private:
        double pos_r;

        rcl_scan_match_backend::ScanMatchBackend* backend;
        rcl_loop_detecter::LoopDetecter* loop_detecter;
        rcl_painter::Painter* painter;
        MapRebuildWorker* map_rebuilder;
        Bridge* bridge = nullptr;
        std::shared_ptr<RosPublisherNode> ros_pub_;
        QThread backend_thread_;
        QThread loop_thread_;
        QThread rebuild_thread_;
    public:
        SlamSystem(Bridge* b, std::shared_ptr<RosPublisherNode> ros_pub = nullptr, double r=0.05, QObject* parent=nullptr);
        ~SlamSystem();

        void setSharedMem(SharedMem* sm);
        bool hasExpectedThreadAffinity() const;
    };
}



#endif // __RCL_SLAM_H__
