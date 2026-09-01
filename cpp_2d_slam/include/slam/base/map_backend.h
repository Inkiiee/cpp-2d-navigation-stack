#ifndef __RCL_MAP_BACKEND_EXECUTER_H__
#define __RCL_MAP_BACKEND_EXECUTER_H__

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>

#include "slam_basic.h"

namespace rcl_map_backend_type{
    using PairHash = rcl_slam_basic_type::PairHash;

    struct Weight{
        int hit_count;
        int miss_count;
        size_t last_seen_frame;
        double x, y;
        Weight();
    };
    struct sub_map{
        std::vector<double> x, y;
        double sensor_x = 0, sensor_y = 0; // 서브맵 생성 시 센서 위치
    };
    using weight_map_index_type = std::pair<int, int>;
    using WeightMap = std::unordered_map<weight_map_index_type, Weight, PairHash>;
}

namespace rcl_map_backend{
    using namespace rcl_map_backend_type;
    using rcl_slam_basic_type::RobotBasePose;

    class MapBackend{
    private:
        double pos_r;
        size_t frame_index = 0;
        WeightMap wm;

        // Occupancy grid map 관련 함수들
        weight_map_index_type get_map_index(double x, double y) const;
        bool isStaticCell(const Weight& weight) const;
        void touchWeightCell(weight_map_index_type index);
        void markFreeRay(weight_map_index_type origin, weight_map_index_type target, const std::unordered_set<weight_map_index_type, PairHash>& hit_cells);
    public:
        MapBackend(double resolution=0.05);
        ~MapBackend();

        // Occupancy grid map 관련 함수들
        void addPos(const std::vector<double>& x, const std::vector<double>& y);
        void updateOccupancyMap(double sensor_x, double sensor_y, const std::vector<double>& x, const std::vector<double>& y);
        void getPos(std::vector<double>& x, std::vector<double>& y, bool static_only = false);
        void getAdjacentPos(const RobotBasePose& pose, std::vector<double>& x, std::vector<double>& y, double radius, bool static_only = false);
        void getAdjacentPosDownsampled(const RobotBasePose& pose, std::vector<double>& x, std::vector<double>& y, double radius, double downsample_res, bool static_only = false);
        void clearMap();
        void swapMapData(MapBackend& other);
        double getResolution() const { return pos_r; }
        void rebuildFromSubmaps(
            const std::vector<double>& sensor_xs, const std::vector<double>& sensor_ys,
            const std::vector<std::vector<double>>& point_xs, const std::vector<std::vector<double>>& point_ys);
        void getOccupancyGridData(std::vector<int8_t>& data, int& width, int& height,
                                  double& origin_x, double& origin_y) const;
        void incrementFrameIndex();
    };
}

#endif // __RCL_MAP_BACKEND_EXECUTER_H__