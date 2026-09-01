#ifndef __RCL_SCAN_MATCH_H__
#define __RCL_SCAN_MATCH_H__

#include <map>
#include <unordered_map>
#include <Eigen/Dense>

#include "slam_basic.h"

namespace rcl_scan_match_type{
    struct Param: public rcl_slam_basic_type::RobotBasePose{
        int iter_count;
        double rmse;
        Param(double ttx=0, double tty=0, double ttheta=0, int iter=0, double r=0);
    };

    // Correlative Scan Matcher (Karto-style) For CSM
    struct LookupTable {
        std::vector<double> data;
        int width, height;
        double resolution;
        double origin_x, origin_y;
        double at(double x, double y) const;
        void set(int gx, int gy, double val);
        double get(int gx, int gy) const;
    };

    // For NDT
    struct cell_info{
        double mean_x;
        double mean_y;
        Eigen::Matrix2d cov_inv;
    };
    struct cell{
        double x, y;
    };

    // Types
    using CellIndexHash = rcl_slam_basic_type::PairHash;
    using cell_index_type = std::pair<int, int>;
    using cell_info_type = std::unordered_map<cell_index_type, cell_info, CellIndexHash>;
    using cells_type = std::unordered_map<cell_index_type, std::vector<cell>, CellIndexHash>;
    using cell_vector = std::vector<cell>;
    using cell_index_vector = std::vector<cell_index_type>;
}

namespace rcl_scan_match{
    using namespace rcl_slam_basic_type;
    using namespace rcl_scan_match_type;
    using namespace rcl_slam_basic_transform;

    class ScanMatcher{
    private:
        // RAII helper: builds PointCloud + KDTree together, auto-deletes tree
        struct CloudTree {
            PointCloud cloud;
            KDTree* tree = nullptr;
            CloudTree(const std::vector<double>& x, const std::vector<double>& y);
            ~CloudTree();
            CloudTree(const CloudTree&) = delete;
            CloudTree& operator=(const CloudTree&) = delete;
        };

        Eigen::MatrixXd rotation(double x, double y, double theta) const;
        void buildKDTree(const PointCloud& cloud, KDTree*& index) const;
        void knnSearch(KDTree* index, const PointCloud& cloud, double query_x, double query_y, int k, std::vector<double>& out_x, std::vector<double>& out_y) const;
    public:
        double cal_rmse(
            std::vector<double>& curr_x, std::vector<double>& curr_y, 
            std::vector<double>& prev_x, std::vector<double>& prev_y, Param p) const;
        double cal_inlier_ratio(
            std::vector<double>& curr_x, std::vector<double>& curr_y, 
            std::vector<double>& prev_x, std::vector<double>& prev_y, double error_cost, Param p) const;
        
        // ─── Iterative Closest Point (Point to Point) ────────────────────────────────
        Param runICP(
            const std::vector<double>& prev_x, const std::vector<double>& prev_y,
            const std::vector<double>& curr_x, const std::vector<double>& curr_y,
            int k = 1, int maxIter = 100, double epsilon = 1e-3) const;
        Param runGauseNewtonICP(
            std::vector<double>& prev_x, std::vector<double>& prev_y,
            std::vector<double>& curr_x, std::vector<double>& curr_y,
            double tx = 0, double ty = 0, double theta = 0,
            int k = 1, int maxIter = 100, double epsilon = 1e-3) const;
        
        // ─── Iterative Closest Point (Point to Plane) ────────────────────────────────
        Param runGauseNewtonICP2(
            std::vector<double>& prev_x, std::vector<double>& prev_y,
            std::vector<double>& curr_x, std::vector<double>& curr_y,
            double tx = 0, double ty = 0, double theta = 0, double step = 0.1,
            int k = 1, int maxIter = 100, double epsilon = 1e-3) const;
        
        // ─── Normal Distributions Transform (NDT) ────────────────────────────────
        bool contain_cell_info(const cell_info_type& info, cell_index_type index) const;
        bool contain_cell(const cells_type& cells, cell_index_type index) const;
        void add_cell(cells_type& cells, cell c, double resolution) const;
        std::pair<int, int> get_cell_index(double x, double y, double resolution) const;
        void get_cells_index(cell_index_vector& index_list, cells_type& cells) const;
        Param runNDT(
            std::vector<double>& prev_x, std::vector<double>& prev_y,
            std::vector<double>& curr_x, std::vector<double>& curr_y,
            double tx = 0, double ty = 0, double theta = 0, double resolution = 1.0, double step = 0.1,
            int maxIter = 100, double epsilon = 1e-3) const;
        // ─── Normal Distributions Transform (Multi-resolution NDT) ────────────────────────────────
        Param runNDTAndGetBestPose(
            std::vector<double>& prev_x, std::vector<double>& prev_y,
            std::vector<double>& curr_x, std::vector<double>& curr_y,
            double tx, double ty, double theta, double step = 0.1,
            int maxIter = 100, double epsilon = 1e-3) const;
        
        // ─── Correlative Scan Matcher (Karto-style) ────────────────────────────────
        LookupTable buildLookupTable(
            const std::vector<double>& ref_x, const std::vector<double>& ref_y,
            double resolution, double smear_sigma) const;
        double scoreCandidate(
            const LookupTable& lut,
            const std::vector<double>& scan_x, const std::vector<double>& scan_y,
            double tx, double ty, double theta) const;
        Param runCSM(
            std::vector<double>& scan_x, std::vector<double>& scan_y,
            std::vector<double>& ref_x, std::vector<double>& ref_y,
            double init_tx, double init_ty, double init_theta,
            double search_xy = 0.5, double search_theta = 0.35,
            double coarse_xy_res = 0.05, double coarse_angle_res = 0.0175,
            double fine_xy_res = 0.005, double fine_angle_res = 0.00175,
            double lut_resolution = 0.02, double smear_sigma = 0.05);
        Param runCSM(
            std::vector<double>& scan_x, std::vector<double>& scan_y,
            const LookupTable& lut,
            double init_tx, double init_ty, double init_theta,
            double search_xy = 0.5, double search_theta = 0.35,
            double coarse_xy_res = 0.05, double coarse_angle_res = 0.0175,
            double fine_xy_res = 0.005, double fine_angle_res = 0.00175);
    };
}

#endif // __RCL_SCAN_MATCH_H__