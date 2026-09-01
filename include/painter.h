#ifndef PAINTER_H
#define PAINTER_H

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QPixmap>
#include <atomic>
#include <memory>
#include <vector>
#include <mutex>

#include "map_backend.h"
#include "my_pose_graph.h"

class SharedMem;
class QMouseEvent;
class RosPublisherNode;

namespace rcl_painter{
    class Painter : public QWidget
    {
        Q_OBJECT
    private:
        double pos_r;
        bool is_lidar_visible = true;
        int paint_frame_counter_ = 0;
        std::atomic<bool> paint_busy_{false};
        QPixmap pixmap, lader_pixmap, world_pixmap, plan_pixmap;
        QLabel* label;
        SharedMem* shared_mem = nullptr;
        std::shared_ptr<RosPublisherNode> ros_pub_;
        rcl_pose_graph::PoseGraph* pose_graph = nullptr;
        rcl_map_backend::MapBackend* world_map = nullptr;
        rcl_map_backend::MapBackend* local_map = nullptr;
        std::mutex* shared_data_mutex = nullptr;

        void drawScan(const std::vector<double>& xs, const std::vector<double>& ys);
        void drawPose(double x, double y, double theta);
        void drawWorldMap();
        void drawGlobalPlan(const std::vector<double>& xs, const std::vector<double>& ys);
    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;
    public:
        explicit Painter(rcl_pose_graph::PoseGraph* pg, rcl_map_backend::MapBackend* wm, rcl_map_backend::MapBackend* lm, double r=0.05, std::mutex* mtx=nullptr, QWidget *parent = nullptr);
        virtual ~Painter();

        void setSharedMem(SharedMem* sm);
        void setRosPublisher(std::shared_ptr<RosPublisherNode> ros_pub);
    public Q_SLOTS:
        void scanUpdate(const std::vector<double>& xs, const std::vector<double>& ys);
        void globalPlanUpdate(const std::vector<double>& xs, const std::vector<double>& ys);
    public Q_SLOTS:
        void predictedPoseUpdate(double x, double y, double theta);
    };
}
#endif
