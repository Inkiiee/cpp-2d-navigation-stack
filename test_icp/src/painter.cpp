#include "painter.h"
#include "ros_publisher_node.hpp"
#include "slam_basic.h"
#include "teleopt.hpp"

#include <algorithm>
#include <chrono>
#include <utility>
#include <QDebug>
#include <QMouseEvent>
#include <QPushButton>

#include <cstdlib>

int map_size_x = 400;
int map_size_y = 400;

namespace {
    constexpr double kMapMin = -16.0;
    constexpr double kMapMax = 16.0;
    constexpr int kMapDrawX = 0;
    constexpr int kMapDrawY = 0;
    constexpr int kMapDrawWidth = 600;
    constexpr int kMapDrawHeight = 600;

    std::pair<double, double> worldToPixel(double wx, double wy) {
        double px = (wx - kMapMin) / (kMapMax - kMapMin) * map_size_x;
        double py = (-wy - kMapMin) / (kMapMax - kMapMin) * map_size_y;
        return {px, py};
    }

    std::pair<double, double> pixelToWorld(double px, double py) {
        double wx = px / map_size_x * (kMapMax - kMapMin) + kMapMin;
        double wy = -(py / map_size_y * (kMapMax - kMapMin) + kMapMin);
        return {wx, wy};
    }
}

namespace rcl_painter{
    using namespace rcl_pose_graph;
    using namespace rcl_map_backend;

    Painter::Painter(PoseGraph* pg, MapBackend* wm, MapBackend* lm, double r, std::mutex* mtx, QWidget *parent):QWidget(parent), pos_r{r}, pixmap(map_size_x, map_size_y), lader_pixmap(map_size_x, map_size_y), world_pixmap(map_size_x, map_size_y), plan_pixmap(map_size_x, map_size_y) {
        pose_graph = pg;
        world_map = wm;
        local_map = lm;
        shared_data_mutex = mtx;

        pixmap.fill(Qt::transparent);
        lader_pixmap.fill(Qt::transparent);
        world_pixmap.fill(Qt::gray);
        plan_pixmap.fill(Qt::transparent);
        QPushButton* btn = new QPushButton("Toggle Lidar", this);
        QObject::connect(btn, &QPushButton::clicked, [this](){
            is_lidar_visible = !is_lidar_visible;
        });
        btn->setGeometry(10, 10, 100, 30);

        label = new QLabel("", this);
        label->setGeometry(120, 10, 200, 30);
    }

    Painter::~Painter(){}

    void Painter::drawScan(const std::vector<double>& xs, const std::vector<double>& ys){
        lader_pixmap.fill(Qt::transparent);
        QPainter painter(&lader_pixmap);
        QPen pen(Qt::green);
        pen.setWidth(1);
        painter.setPen(pen);

        for(size_t i=0; i<xs.size(); i++){
            auto [mx, my] = worldToPixel(xs[i], ys[i]);
            painter.drawPoint(mx, my);
        }
        update();
    }
    void Painter::drawPose(double x, double y, double theta){
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        QPen pen(Qt::red);
        pen.setWidth(1);
        painter.setPen(pen);

        std::vector<rcl_slam_basic_type::RobotBasePose> poses;
        if(shared_data_mutex){
            std::lock_guard<std::mutex> lock(*shared_data_mutex);
            auto snapshot = pose_graph->getPoseSnapshot();
            poses.assign(snapshot.begin(), snapshot.end());
        }
        for(const auto& p : poses){
            auto [mx, my] = worldToPixel(p.tx, p.ty);
            painter.drawPoint(mx, my);
        }

        pen.setColor(Qt::blue);
        pen.setWidth(2);
        painter.setPen(pen);
        // 현재 위치 강조
        static std::vector<double> arrow_x{-2, -1, 0, 1, 2, 2, 2,  2,  2, 3, 3,  3, 4};
        static std::vector<double> arrow_y{ 0,  0, 0, 0, 2, 1, 0, -1, -2, 1, 0, -1, 0};

        for(size_t i=0; i<arrow_x.size(); i++){
            double mx = (arrow_x[i]) * pos_r * 3;
            double my = (arrow_y[i]) * pos_r * 3;
            double rx = std::cos(theta) * mx - std::sin(theta) * my + x;
            double ry = std::sin(theta) * mx + std::cos(theta) * my + y;
            auto [px, py] = worldToPixel(rx, ry);
            painter.drawPoint(px, py);
        }
        update();
    }

    void Painter::drawWorldMap(){
        world_pixmap.fill(Qt::gray);
        QPainter painter(&world_pixmap);
        QPen pen(Qt::black);
        pen.setWidth(1);
        painter.setPen(pen);

        std::vector<double> x, y;
        if(shared_data_mutex){
            std::lock_guard<std::mutex> lock(*shared_data_mutex);
            world_map->getPos(x, y, true);
        } else {
            world_map->getPos(x, y, true);
        }
        for(size_t i=0; i<x.size(); i++){
            auto [mx, my] = worldToPixel(x[i], y[i]);
            painter.drawPoint(mx, my);
        }
        update();
    }

    void Painter::drawGlobalPlan(const std::vector<double>& xs, const std::vector<double>& ys){
        plan_pixmap.fill(Qt::transparent);
        QPainter painter(&plan_pixmap);
        QPen pen(Qt::yellow);
        pen.setWidth(2);
        painter.setPen(pen);

        const size_t count = std::min(xs.size(), ys.size());
        for(size_t i=1; i<count; i++){
            auto [prev_x, prev_y] = worldToPixel(xs[i - 1], ys[i - 1]);
            auto [curr_x, curr_y] = worldToPixel(xs[i], ys[i]);
            painter.drawLine(QPointF(prev_x, prev_y), QPointF(curr_x, curr_y));
        }

        for(size_t i=0; i<count; i++){
            auto [px, py] = worldToPixel(xs[i], ys[i]);
            painter.drawPoint(QPointF(px, py));
        }
        update();
    }

    void Painter::paintEvent(QPaintEvent* event){
        if(shared_mem){
            label->setText(QString("speed: %1, theta: %2").arg(shared_mem->get_speed()).arg(shared_mem->get_theta()));
        }

        QPainter painter(this);
        painter.drawPixmap(kMapDrawX, kMapDrawY, kMapDrawWidth, kMapDrawHeight, world_pixmap);
        if(is_lidar_visible) painter.drawPixmap(kMapDrawX, kMapDrawY, kMapDrawWidth, kMapDrawHeight, lader_pixmap);
        painter.drawPixmap(kMapDrawX, kMapDrawY, kMapDrawWidth, kMapDrawHeight, plan_pixmap);
        painter.drawPixmap(kMapDrawX, kMapDrawY, kMapDrawWidth, kMapDrawHeight, pixmap);

        QWidget::paintEvent(event);
    }

    void Painter::mousePressEvent(QMouseEvent* event){
        if(event->button() == Qt::LeftButton){
            const QPointF click_pos = event->position();
            const bool in_map =
                click_pos.x() >= kMapDrawX &&
                click_pos.x() <= kMapDrawX + kMapDrawWidth &&
                click_pos.y() >= kMapDrawY &&
                click_pos.y() <= kMapDrawY + kMapDrawHeight;

            if(in_map){
                const double px = (click_pos.x() - kMapDrawX) / kMapDrawWidth * map_size_x;
                const double py = (click_pos.y() - kMapDrawY) / kMapDrawHeight * map_size_y;
                auto [map_x, map_y] = pixelToWorld(px, py);

                if(ros_pub_){
                    ros_pub_->publishGoal(map_x, map_y);
                    qDebug() << "[goal] publish /goal map x=" << map_x << "y=" << map_y;
                } else {
                    qDebug() << "[goal] skipped: RosPublisherNode is not connected";
                }
                event->accept();
                return;
            }
        }

        QWidget::mousePressEvent(event);
    }

    void Painter::setSharedMem(SharedMem* sm){
        shared_mem = sm;
    } 

    void Painter::setRosPublisher(std::shared_ptr<RosPublisherNode> ros_pub){
        ros_pub_ = ros_pub;
    }

    void Painter::scanUpdate(const std::vector<double>& xs, const std::vector<double>& ys){
        // Painter는 main thread에서 실행 — 이전 그리기가 끝나지 않았으면 드롭
        bool expected = false;
        if(!paint_busy_.compare_exchange_strong(expected, true)){
            return;
        }
        drawScan(xs, ys);
        repaint();
        paint_busy_ = false;
    }
    void Painter::globalPlanUpdate(const std::vector<double>& xs, const std::vector<double>& ys){
        drawGlobalPlan(xs, ys);
        repaint();
    }
    void Painter::predictedPoseUpdate(double x, double y, double theta){
        auto t0 = std::chrono::steady_clock::now();
        // Throttle expensive world map redraws (every 5 frames)
        if(++paint_frame_counter_ % 5 == 0){
            drawWorldMap();
        }
        auto t1 = std::chrono::steady_clock::now();
        drawPose(x, y, theta);
        auto t2 = std::chrono::steady_clock::now();
        repaint();
        auto t3 = std::chrono::steady_clock::now();

        auto us_world = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        auto us_pose = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        auto us_repaint = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
        if(paint_frame_counter_ % 20 == 0){
            qDebug() << "[TIMING painter] worldMap=" << us_world << "us drawPose=" << us_pose
                     << "us repaint=" << us_repaint << "us total=" << (us_world + us_pose + us_repaint) << "us";
        }
    }
}
