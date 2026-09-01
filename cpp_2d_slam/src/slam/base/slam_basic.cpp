#include "slam_basic.h"

#include <cmath>

namespace rcl_slam_basic_type{
    bool Point::operator<(const Point& t) const{
        if(x != t.x) return x < t.x;
        return y < t.y;
    }
    Point::Point(const Point& p):x{p.x}, y{p.y}{}
    Point::Point(double xx, double yy):x{xx}, y{yy}{}

    RobotBasePose::RobotBasePose(double x, double y, double t):tx{x}, ty{y}, theta{t}{}
    RobotBasePose::RobotBasePose(const RobotBasePose& p):tx{p.tx}, ty{p.ty}, theta{p.theta}{}
}

namespace rcl_slam_basic_transform{
    void rotationAndTranslation(double tx, double ty, double theta, std::vector<double>& x, std::vector<double>& y){
        double cos_t = std::cos(theta), sin_t = std::sin(theta);
        for(size_t i=0; i<x.size(); i++){
            double rx = cos_t * x[i] - sin_t * y[i] + tx;
            double ry = sin_t * x[i] + cos_t * y[i] + ty;
            x[i] = rx; y[i] = ry;
        }
    }

    RobotBasePose identityPose(){
        return RobotBasePose();
    }

    RobotBasePose inversePose(const RobotBasePose& p){
        RobotBasePose inv;
        inv.tx = -(std::cos(p.theta) * p.tx + std::sin(p.theta) * p.ty);
        inv.ty = -(-std::sin(p.theta) * p.tx + std::cos(p.theta) * p.ty);
        inv.theta = -p.theta;

        return inv;
    }

    RobotBasePose relativePose(const RobotBasePose& from, const RobotBasePose& to){
        double dx = to.tx - from.tx;
        double dy = to.ty - from.ty;
        double dtheta = normalizeAngle(to.theta - from.theta);

        RobotBasePose relative;
        relative.tx = std::cos(from.theta) * dx + std::sin(from.theta) * dy;
        relative.ty = -std::sin(from.theta) * dx + std::cos(from.theta) * dy;
        relative.theta = normalizeAngle(dtheta);

        return relative;
    }

    double normalizeAngle(double a){
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
}