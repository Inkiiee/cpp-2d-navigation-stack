#include "scan_match.h"

#include <cmath>
#include <vector>

namespace rcl_scan_match_type{
    Param::Param(double ttx, double tty, double ttheta, int iter, double r):
        RobotBasePose(ttx, tty, ttheta), iter_count(iter), rmse(r){}

    double LookupTable::at(double x, double y) const {
        int gx = static_cast<int>(std::round((x - origin_x) / resolution));
        int gy = static_cast<int>(std::round((y - origin_y) / resolution));
        return get(gx, gy);
    }

    void LookupTable::set(int gx, int gy, double val) {
        if (gx >= 0 && gx < width && gy >= 0 && gy < height)
            data[gy * width + gx] = std::max(data[gy * width + gx], val);
    }

    double LookupTable::get(int gx, int gy) const {
        if (gx >= 0 && gx < width && gy >= 0 && gy < height)
            return data[gy * width + gx];
        return 0.0;
    }
}

namespace rcl_scan_match{
    ScanMatcher::CloudTree::CloudTree(const std::vector<double>& x, const std::vector<double>& y) {
        cloud.pts.reserve(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            cloud.pts.push_back(Point{x[i], y[i]});
        }
        tree = new KDTree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();
    }

    ScanMatcher::CloudTree::~CloudTree() {
        delete tree;
    }

    double ScanMatcher::cal_rmse(
        std::vector<double>& curr_x, std::vector<double>& curr_y, 
        std::vector<double>& prev_x, std::vector<double>& prev_y, Param p) const
    {
        int N = std::min(prev_x.size(), curr_x.size());
        CloudTree ct(curr_x, curr_y);
        double rmse = 0;

        for(int i=0; i<N; i++){
            double x = std::cos(p.theta) * prev_x[i] - std::sin(p.theta) * prev_y[i] + p.tx;
            double y = std::sin(p.theta) * prev_x[i] + std::cos(p.theta) * prev_y[i] + p.ty;

            std::vector<double> nx, ny;
            knnSearch(ct.tree, ct.cloud, x, y, 1, nx, ny);

            double dx = x - nx[0];
            double dy = y - ny[0];
            rmse += (dx * dx + dy * dy);
        }

        return std::sqrt(rmse / N);
    }

    double ScanMatcher::cal_inlier_ratio(
        std::vector<double>& curr_x, std::vector<double>& curr_y,
        std::vector<double>& prev_x, std::vector<double>& prev_y, double error_cost, Param p) const
    {
        size_t N = std::min(prev_x.size(), curr_x.size());
        CloudTree ct(curr_x, curr_y);
        int count = 0;

        for(size_t i=0; i<N; i++){
            double x = std::cos(p.theta) * prev_x[i] - std::sin(p.theta) * prev_y[i] + p.tx;
            double y = std::sin(p.theta) * prev_x[i] + std::cos(p.theta) * prev_y[i] + p.ty;

            std::vector<double> nx, ny;
            knnSearch(ct.tree, ct.cloud, x, y, 1, nx, ny);

            double dx = x - nx[0];
            double dy = y - ny[0];
            double distance = std::sqrt(dx * dx + dy * dy);
            if(distance <= error_cost) count++;
        }

        return ((double)count) / N;
    }

    Eigen::MatrixXd ScanMatcher::rotation(double x, double y, double theta) const {
        Eigen::MatrixXd rot(2, 2);
        rot(0, 0) = std::cos(theta); rot(0,1) = - std::sin(theta);
        rot(1, 0) = std::sin(theta); rot(1,1) = std::cos(theta);
        Eigen::MatrixXd pos(2, 1);
        pos(0, 0) = x; pos(1, 0) = y;
        return rot*pos;
    }

    void ScanMatcher::buildKDTree(const PointCloud& cloud, KDTree*& index) const {
        index = new KDTree(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index->buildIndex();
    }

    void ScanMatcher::knnSearch(
        KDTree* index, const PointCloud& cloud, 
        double query_x, double query_y, int k, 
        std::vector<double>& out_x, std::vector<double>& out_y) const
    {
        double query[2] = {query_x, query_y};
        std::vector<size_t> ret_index(k);
        std::vector<double> out_dist_sqr(k);
        nanoflann::KNNResultSet<double> resultSet(k);
        resultSet.init(&ret_index[0], &out_dist_sqr[0]);
        index->findNeighbors(resultSet, query, nanoflann::SearchParams(10));
        for(int i=0; i < k; i ++){
            out_x.push_back(cloud.pts[ret_index[i]].x);
            out_y.push_back(cloud.pts[ret_index[i]].y);
        }
    }

    // ─── Iterative Closest Point (Point to Point) ────────────────────────────────
    Param ScanMatcher::runICP(
    const std::vector<double>& prev_x, const std::vector<double>& prev_y,
    const std::vector<double>& curr_x, const std::vector<double>& curr_y,
    int k, int maxIter, double epsilon) const
    {
        int iter = 0;
        int N = std::min(prev_x.size(), curr_x.size());
        CloudTree ct(curr_x, curr_y);

        std::vector<double> A_x = prev_x; // Source (previous scan)
        std::vector<double> A_y = prev_y;

        Eigen::Matrix3d T_total = Eigen::Matrix3d::Identity();
        double prevError = 1e12;
        for (iter = 0; iter < maxIter; ++iter) {
            std::vector<double> B_x(N), B_y(N);
            for (int i = 0; i < N; ++i) {
                std::vector<double> nx, ny;
                knnSearch(ct.tree, ct.cloud, A_x[i], A_y[i], k, nx, ny);
                double mx = 0, my = 0;
                for (int j = 0; j < k; ++j) {
                    mx += nx[j]; my += ny[j];
                }
                B_x[i] = mx / k;
                B_y[i] = my / k;
            }

            double meanA_x = 0, meanA_y = 0, meanB_x = 0, meanB_y = 0;
            for (int i = 0; i < N; ++i) {
                meanA_x += A_x[i]; meanA_y += A_y[i];
                meanB_x += B_x[i]; meanB_y += B_y[i];
            }
            meanA_x /= N; meanA_y /= N;
            meanB_x /= N; meanB_y /= N;

            Eigen::MatrixXd AA(2, N), BB(2, N);
            for (int i = 0; i < N; ++i) {
                AA(0,i) = A_x[i] - meanA_x;
                AA(1,i) = A_y[i] - meanA_y;
                BB(0,i) = B_x[i] - meanB_x;
                BB(1,i) = B_y[i] - meanB_y;
            }

            Eigen::Matrix2d H = AA * BB.transpose();

            Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

            if (R.determinant() < 0) {
                Eigen::Matrix2d V = svd.matrixV();
                V.col(1) *= -1;
                R = V * svd.matrixU().transpose();
            }

            Eigen::Vector2d t = Eigen::Vector2d(meanB_x, meanB_y) - R * Eigen::Vector2d(meanA_x, meanA_y);

            Eigen::Matrix3d T_curr = Eigen::Matrix3d::Identity();
            T_curr.block<2,2>(0,0) = R;
            T_curr.block<2,1>(0,2) = t;

            T_total = T_curr * T_total;

            for (int i = 0; i < N; ++i) {
                Eigen::Vector3d p(A_x[i], A_y[i], 1.0);
                Eigen::Vector3d p_new = T_curr * p;
                A_x[i] = p_new(0);
                A_y[i] = p_new(1);
            }

            double error = 0;
            for (int i = 0; i < N; ++i) {
                double dx = B_x[i] - A_x[i];
                double dy = B_y[i] - A_y[i];
                error += std::sqrt(dx*dx + dy*dy);
            }
            error /= N;

            if (iter > 0 && std::abs(prevError - error) / prevError < epsilon){
                iter++;
                break;
            }
            else if(error < (epsilon * 0.01)){
                iter++;
                break;
            }
            prevError = error;
        }

        double theta = std::atan2(T_total(1,0), T_total(0,0));

        Param p;
        p.iter_count = iter;
        p.tx = T_total(0, 2);
        p.ty = T_total(1, 2);
        p.theta = (std::abs(theta) >= 0.5235983) ? 0 : theta;

        return p;
    }

    Param ScanMatcher::runGauseNewtonICP(
        std::vector<double>& prev_x, std::vector<double>& prev_y,
        std::vector<double>& curr_x, std::vector<double>& curr_y,
        double tx, double ty, double theta,
        int k, int maxIter, double epsilon) const 
    {
        int iter = 0;
        int N = std::min(prev_x.size(), curr_x.size());
        CloudTree ct(curr_x, curr_y);

        for (iter = 0; iter < maxIter; ++iter) {
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            double cos_t = std::cos(theta), sin_t = std::sin(theta);

            std::vector<double> B_x(N), B_y(N);
            std::vector<double> nx, ny;
            for (int i = 0; i < N; ++i) {
                double x = cos_t * prev_x[i] - sin_t * prev_y[i] + tx;
                double y = sin_t * prev_x[i] + cos_t * prev_y[i] + ty;

                nx.clear(); ny.clear();
                knnSearch(ct.tree, ct.cloud, x, y, k, nx, ny);
                double mx = 0, my = 0;
                for (int j = 0; j < k; ++j) {
                    mx += nx[j]; my += ny[j];
                }
                B_x[i] = mx / k;
                B_y[i] = my / k;

                Eigen::Matrix<double, 2, 3> Ji;
                Ji(0, 0) = 1; Ji(0, 1) = 0; Ji(0, 2) = -(sin_t * prev_x[i] + cos_t * prev_y[i]);
                Ji(1, 0) = 0; Ji(1, 1) = 1; Ji(1, 2) = (cos_t * prev_x[i] - sin_t * prev_y[i]);
                Eigen::Vector2d ei(x - B_x[i], y - B_y[i]);
                H.noalias() += Ji.transpose() * Ji;
                b.noalias() += Ji.transpose() * ei;
            }

            Eigen::Vector3d delta = -H.ldlt().solve(b);
            if(delta.norm() < epsilon){
                iter++;
                break;
            }
            tx += delta(0, 0); ty += delta(1, 0), theta += delta(2, 0);
        }

        Param p;
        p.tx = tx;
        p.ty = ty;
        p.theta = theta;
        p.iter_count = iter;

        return p;
    }

    // ─── Iterative Closest Point (Point to Plane) ────────────────────────────────
    Param ScanMatcher::runGauseNewtonICP2(
        std::vector<double>& prev_x, std::vector<double>& prev_y,
        std::vector<double>& curr_x, std::vector<double>& curr_y,
        double tx, double ty, double theta, double step,
        int k, int maxIter, double epsilon) const
    {
        int iter = 0;
        int N = std::min(prev_x.size(), curr_x.size());
        CloudTree ct(curr_x, curr_y);

        for (iter = 0; iter < maxIter; ++iter) {
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            double cos_t = std::cos(theta), sin_t = std::sin(theta);

            std::vector<double> B_x(N), B_y(N);
            std::vector<double> nx, ny;
            for (int i = 0; i < N; ++i) {
                double x = cos_t * prev_x[i] - sin_t * prev_y[i] + tx;
                double y = sin_t * prev_x[i] + cos_t * prev_y[i] + ty;

                nx.clear(); ny.clear();
                knnSearch(ct.tree, ct.cloud, x, y, k, nx, ny);
                double mx = 0, my = 0;
                for (int j = 0; j < k; ++j) {
                    mx += nx[j]; my += ny[j];
                }
                B_x[i] = mx / k;
                B_y[i] = my / k;

                double cov_xx = 0, cov_xy = 0, cov_yy = 0;
                for(int j=0; j < k; j++){
                    double dx = nx[j] - B_x[i];
                    double dy = ny[j] - B_y[i];

                    cov_xx += dx * dx;
                    cov_xy += dx * dy;
                    cov_yy += dy * dy;
                }

                Eigen::Matrix2d cov;
                cov<<cov_xx, cov_xy,
                    cov_xy, cov_yy;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
                Eigen::Vector2d normal = solver.eigenvectors().col(0); 

                if(normal.norm() < 1e-6) continue;

                double n_x = normal(0);
                double n_y = normal(1);

                Eigen::Matrix<double, 1, 3> Ji;
                Ji(0, 0) = n_x;
                Ji(0, 1) = n_y;
                Ji(0, 2) = (-sin_t * prev_x[i] - cos_t * prev_y[i]) * n_x + (cos_t * prev_x[i] - sin_t * prev_y[i]) * n_y;
                double ei = n_x * (x - B_x[i]) + n_y * (y - B_y[i]);
                H.noalias() += Ji.transpose() * Ji;
                b.noalias() += Ji.transpose() * ei;
            }

            Eigen::Vector3d delta = -H.ldlt().solve(b);
            if(delta.norm() < epsilon){
                iter++;
                break;
            }

            for(int i=0; i<3; i++){
                double num = delta(i, 0);
                if(std::abs(num) > step)
                    delta(i, 0) = (num < 0) ? -step : step;
            }

            tx += delta(0, 0); ty += delta(1, 0), theta += delta(2, 0);
        }

        Param p;
        p.tx = tx;
        p.ty = ty;
        p.theta = theta;
        p.iter_count = iter;

        return p;
    }

    // ─── Normal Distributions Transform (NDT) ────────────────────────────────
    bool ScanMatcher::contain_cell_info(const cell_info_type& info, cell_index_type index) const {
        if(info.find(index) == info.end())
            return false;
        return true;
    }

    bool ScanMatcher::contain_cell(const cells_type& cells, cell_index_type index) const {
        if(cells.find(index) == cells.end())
            return false;
        return true;
    }

    void ScanMatcher::get_cells_index(cell_index_vector& index_list, cells_type& cells) const {
        cells_type::const_iterator it;

        for(it = cells.begin(); it != cells.end(); it++){
            index_list.push_back(it->first);
        }
    }

    void ScanMatcher::add_cell(cells_type& cells, cell c, double resolution) const {
        cell_index_type index = get_cell_index(c.x, c.y, resolution);
        cells[index].push_back(c);
    }

    std::pair<int, int> ScanMatcher::get_cell_index(double x, double y, double resolution) const {
        int cell_x = static_cast<int>(std::ceil(x / resolution));
        int cell_y = static_cast<int>(std::ceil(y / resolution));

        return {cell_x, cell_y};
    }

    Param ScanMatcher::runNDT(
        std::vector<double>& prev_x, std::vector<double>& prev_y,
        std::vector<double>& curr_x, std::vector<double>& curr_y,
        double tx, double ty, double theta, double resolution, double step,
        int maxIter, double epsilon) const
    {
        int N = std::min(prev_x.size(), curr_x.size());
        cells_type cells;
        cell_info_type cells_info;

        for(size_t i=0; i<curr_x.size(); i++){
            cell c = {curr_x[i], curr_y[i]};
            add_cell(cells, c, resolution);
        }

        cell_index_vector index_list;
        get_cells_index(index_list, cells);

        for(size_t c=0; c<index_list.size(); c++){
            cell_index_type key = index_list[c];
            const cell_vector& grid_cell = cells[key];

            double mean_x = 0, mean_y = 0;
            int size = grid_cell.size();
            if(size < 2) continue;

            for(int i=0; i<size; i++){
                mean_x += grid_cell[i].x;
                mean_y += grid_cell[i].y;
            }
            mean_x/=size; mean_y/=size;

            Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
            for(int i=0; i<size; i++){
                Eigen::Vector2d pos(grid_cell[i].x - mean_x, grid_cell[i].y - mean_y);
                cov.noalias() += (pos * pos.transpose());
            }
            cov /= (size - 1);
            if(cov.determinant() < 1e-6) continue;

            cell_info info;
            info.cov_inv = cov.inverse();
            info.mean_x = mean_x;
            info.mean_y = mean_y;

            cells_info[key] = info;
        }

        int iter;
        for(iter = 0; iter < maxIter; iter++){
            Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            double sin_theta = std::sin(theta);
            double cos_theta = std::cos(theta);
            for(int i=0; i<N; i++){
                double px = prev_x[i];
                double py = prev_y[i];

                double x = cos_theta * px - sin_theta * py + tx;
                double y = sin_theta * px + cos_theta * py + ty;

                Eigen::Matrix<double, 2, 3> Ji;
                Ji(0, 0) = -sin_theta * px - cos_theta * py;
                Ji(0, 1) = 1;
                Ji(0, 2) = 0;

                Ji(1, 0) =  cos_theta * px - sin_theta * py;
                Ji(1, 1) = 0;
                Ji(1, 2) = 1;

                cell_index_type cell_index = get_cell_index(x, y, resolution);
                auto it = cells_info.find(cell_index);
                if(it == cells_info.end()) continue;

                const cell_info& info = it->second;
                Eigen::Vector2d r(x - info.mean_x, y - info.mean_y);

                H += (Ji.transpose() * info.cov_inv * Ji);
                b += (Ji.transpose() * info.cov_inv * r);
            }

            Eigen::Vector3d delta = H.ldlt().solve(b);
            if (delta.norm() > 1.0) delta *= 1.0 / delta.norm(); // clip
            if(delta.norm() < epsilon){ iter++; break;}

            double theta_limit = step * 0.3;
            if(std::abs(delta(0, 0)) > theta_limit)
                delta(0, 0) = (delta(0, 0) < 0) ? -theta_limit : theta_limit;
            for(int i=1; i<3; i++){
                double num = delta(i, 0);
                if(std::abs(num) > step)
                    delta(i, 0) = (num < 0) ? -step : step;
            }

            theta -= delta(0, 0); tx -= delta(1, 0); ty -= delta(2, 0);

        }

        Param p;
        p.tx = tx; p.ty = ty; p.theta = theta; p.iter_count = iter;

        return p;
    }

    // ─── Normal Distributions Transform (Multi-resolution NDT) ────────────────────────────────
    Param ScanMatcher::runNDTAndGetBestPose(
        std::vector<double>& prev_x, std::vector<double>& prev_y,
        std::vector<double>& curr_x, std::vector<double>& curr_y,
        double tx, double ty, double theta, double step,
        int maxIter, double epsilon) const
    {
        Param origin_p;
        origin_p.tx = tx; origin_p.ty = ty; origin_p.theta = theta;

        static double resolutions[] = {1.0, 0.5, 0.3, 0.1, 0.05};
        Param p{tx, ty, theta, 0};
        double rmse_origin = cal_rmse(curr_x, curr_y, prev_x, prev_y, origin_p);
        for(int i=0; i<5; i++){
            p = runNDT(prev_x, prev_y, curr_x, curr_y, p.tx, p.ty, p.theta, resolutions[i], step, maxIter, epsilon);
            double rmse_ndt = cal_rmse(curr_x, curr_y, prev_x, prev_y, p);

            if(rmse_ndt < rmse_origin){
                p.theta = normalizeAngle(p.theta);
                p.rmse = rmse_ndt;
                return p;
            }
        }
        origin_p.rmse = rmse_origin;
        return origin_p;
    }

    // ─── Correlative Scan Matcher (Karto-style) ────────────────────────────────
    LookupTable ScanMatcher::buildLookupTable(
        const std::vector<double>& ref_x, const std::vector<double>& ref_y,
        double resolution, double smear_sigma) const
    {
        if (ref_x.empty()) return LookupTable{};

        // Compute bounding box with margin
        double margin = smear_sigma * 4.0 + 1.0;
        double min_x = *std::min_element(ref_x.begin(), ref_x.end()) - margin;
        double min_y = *std::min_element(ref_y.begin(), ref_y.end()) - margin;
        double max_x = *std::max_element(ref_x.begin(), ref_x.end()) + margin;
        double max_y = *std::max_element(ref_y.begin(), ref_y.end()) + margin;

        LookupTable lut;
        lut.resolution = resolution;
        lut.origin_x = min_x;
        lut.origin_y = min_y;
        lut.width  = static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1;
        lut.height = static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1;
        lut.data.assign(static_cast<size_t>(lut.width) * lut.height, 0.0);

        // Gaussian kernel radius in cells
        int kernel_radius = static_cast<int>(std::ceil(smear_sigma * 3.0 / resolution));
        double inv_2sigma2 = 1.0 / (2.0 * smear_sigma * smear_sigma);

        // Precompute Gaussian kernel
        int kernel_size = 2 * kernel_radius + 1;
        std::vector<double> kernel(kernel_size * kernel_size);
        for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
            for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
                double dist2 = (dx * resolution) * (dx * resolution) + (dy * resolution) * (dy * resolution);
                kernel[(dy + kernel_radius) * kernel_size + (dx + kernel_radius)] = std::exp(-dist2 * inv_2sigma2);
            }
        }

        // Stamp each reference point with the precomputed kernel
        for (size_t i = 0; i < ref_x.size(); ++i) {
            int cx = static_cast<int>(std::round((ref_x[i] - min_x) / resolution));
            int cy = static_cast<int>(std::round((ref_y[i] - min_y) / resolution));

            for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
                for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
                    double val = kernel[(dy + kernel_radius) * kernel_size + (dx + kernel_radius)];
                    lut.set(cx + dx, cy + dy, val);
                }
            }
        }

        return lut;
    }

    double ScanMatcher::scoreCandidate(
        const LookupTable& lut,
        const std::vector<double>& scan_x, const std::vector<double>& scan_y,
        double tx, double ty, double theta) const
    {
        double score = 0.0;
        double cos_t = std::cos(theta);
        double sin_t = std::sin(theta);

        for (size_t i = 0; i < scan_x.size(); ++i) {
            double wx = cos_t * scan_x[i] - sin_t * scan_y[i] + tx;
            double wy = sin_t * scan_x[i] + cos_t * scan_y[i] + ty;
            score += lut.at(wx, wy);
        }

        return score;
    }

    // ── Optimized CSM: pre-rotation per θ + integer grid + coarse scan downsampling ──
    Param ScanMatcher::runCSM(
        std::vector<double>& scan_x, std::vector<double>& scan_y,
        const LookupTable& lut,
        double init_tx, double init_ty, double init_theta,
        double search_xy, double search_theta,
        double coarse_xy_res, double coarse_angle_res,
        double fine_xy_res, double fine_angle_res)
    {
        Param best;
        best.tx = init_tx;
        best.ty = init_ty;
        best.theta = init_theta;

        if (scan_x.empty() || lut.data.empty()) return best;

        const size_t N = scan_x.size();
        const double inv_res = 1.0 / lut.resolution;
        const int W = lut.width, H = lut.height;
        const double ox = lut.origin_x, oy = lut.origin_y;
        const double* lut_data = lut.data.data();

        // ── Downsample scan for coarse pass (every 3rd point) ──
        constexpr size_t kCoarseStride = 3;
        const size_t N_coarse = (N + kCoarseStride - 1) / kCoarseStride;
        std::vector<double> cscan_x(N_coarse), cscan_y(N_coarse);
        for (size_t i = 0, j = 0; i < N && j < N_coarse; i += kCoarseStride, ++j) {
            cscan_x[j] = scan_x[i];
            cscan_y[j] = scan_y[i];
        }

        std::vector<int> grid_x(N), grid_y(N);
        std::vector<int> coarse_grid_x(N_coarse), coarse_grid_y(N_coarse);

        // ── Pass 1: Coarse search ──
        double best_score = -1.0;

        for (double dtheta = -search_theta; dtheta <= search_theta; dtheta += coarse_angle_res) {
            double cand_theta = init_theta + dtheta;
            double cos_t = std::cos(cand_theta);
            double sin_t = std::sin(cand_theta);

            // Pre-rotate downsampled scan → integer grid coords
            for (size_t i = 0; i < N_coarse; ++i) {
                coarse_grid_x[i] = static_cast<int>(std::round((cos_t * cscan_x[i] - sin_t * cscan_y[i]) * inv_res));
                coarse_grid_y[i] = static_cast<int>(std::round((sin_t * cscan_x[i] + cos_t * cscan_y[i]) * inv_res));
            }

            for (double dx = -search_xy; dx <= search_xy; dx += coarse_xy_res) {
                int off_x = static_cast<int>(std::round((init_tx + dx - ox) * inv_res));
                for (double dy = -search_xy; dy <= search_xy; dy += coarse_xy_res) {
                    int off_y = static_cast<int>(std::round((init_ty + dy - oy) * inv_res));

                    double score = 0.0;
                    for (size_t i = 0; i < N_coarse; ++i) {
                        int gx = coarse_grid_x[i] + off_x;
                        int gy = coarse_grid_y[i] + off_y;
                        if (static_cast<unsigned>(gx) < static_cast<unsigned>(W) &&
                            static_cast<unsigned>(gy) < static_cast<unsigned>(H))
                            score += lut_data[gy * W + gx];
                    }
                    if (score > best_score) {
                        best_score = score;
                        best.tx = init_tx + dx;
                        best.ty = init_ty + dy;
                        best.theta = cand_theta;
                    }
                }
            }
        }

        // ── Pass 2: Fine search around coarse best (full scan points) ──
        double fine_tx = best.tx;
        double fine_ty = best.ty;
        double fine_theta = best.theta;
        best_score = -1.0;

        for (double dtheta = -coarse_angle_res; dtheta <= coarse_angle_res; dtheta += fine_angle_res) {
            double cand_theta = fine_theta + dtheta;
            double cos_t = std::cos(cand_theta);
            double sin_t = std::sin(cand_theta);

            // Pre-rotate all scan points → integer grid coords
            for (size_t i = 0; i < N; ++i) {
                grid_x[i] = static_cast<int>(std::round((cos_t * scan_x[i] - sin_t * scan_y[i]) * inv_res));
                grid_y[i] = static_cast<int>(std::round((sin_t * scan_x[i] + cos_t * scan_y[i]) * inv_res));
            }

            for (double dx = -coarse_xy_res; dx <= coarse_xy_res; dx += fine_xy_res) {
                int off_x = static_cast<int>(std::round((fine_tx + dx - ox) * inv_res));
                for (double dy = -coarse_xy_res; dy <= coarse_xy_res; dy += fine_xy_res) {
                    int off_y = static_cast<int>(std::round((fine_ty + dy - oy) * inv_res));

                    double score = 0.0;
                    for (size_t i = 0; i < N; ++i) {
                        int gx = grid_x[i] + off_x;
                        int gy = grid_y[i] + off_y;
                        if (static_cast<unsigned>(gx) < static_cast<unsigned>(W) &&
                            static_cast<unsigned>(gy) < static_cast<unsigned>(H))
                            score += lut_data[gy * W + gx];
                    }
                    if (score > best_score) {
                        best_score = score;
                        best.tx = fine_tx + dx;
                        best.ty = fine_ty + dy;
                        best.theta = cand_theta;
                    }
                }
            }
        }

        best.theta = normalizeAngle(best.theta);
        best.rmse = (N > 0) ? (1.0 - best_score / static_cast<double>(N)) : 999.0;

        return best;
    }

    // Original overload: builds LUT then delegates to optimized version
    Param ScanMatcher::runCSM(
        std::vector<double>& scan_x, std::vector<double>& scan_y,
        std::vector<double>& ref_x, std::vector<double>& ref_y,
        double init_tx, double init_ty, double init_theta,
        double search_xy, double search_theta,
        double coarse_xy_res, double coarse_angle_res,
        double fine_xy_res, double fine_angle_res,
        double lut_resolution, double smear_sigma)
    {
        if (scan_x.empty() || ref_x.empty()) {
            Param p; p.tx = init_tx; p.ty = init_ty; p.theta = init_theta; return p;
        }
        LookupTable lut = buildLookupTable(ref_x, ref_y, lut_resolution, smear_sigma);
        return runCSM(scan_x, scan_y, lut, init_tx, init_ty, init_theta,
                      search_xy, search_theta, coarse_xy_res, coarse_angle_res,
                      fine_xy_res, fine_angle_res);
    }
}
