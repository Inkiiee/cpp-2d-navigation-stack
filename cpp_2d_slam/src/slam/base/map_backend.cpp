#include "map_backend.h"

#include <cmath>
#include <algorithm>

namespace rcl_map_backend_type{
    Weight::Weight():hit_count(0), miss_count(0), last_seen_frame(0), x(0), y(0){}
}

namespace rcl_map_backend{
    MapBackend::MapBackend(double resolution): pos_r(resolution) {}
    MapBackend::~MapBackend(){}

    weight_map_index_type MapBackend::get_map_index(double x, double y) const{
        int cell_x = static_cast<int>(std::ceil(x / pos_r));
        int cell_y = static_cast<int>(std::ceil(y / pos_r));
        return {cell_x, cell_y};
    }

    void MapBackend::touchWeightCell(weight_map_index_type index){
        if(wm.find(index) == wm.end()){
            Weight w;
            w.x = index.first * pos_r;
            w.y = index.second * pos_r;
            wm[index] = w;
        }
    }

    void MapBackend::addPos(const std::vector<double>& x, const std::vector<double>& y){
        for(size_t i=0; i<x.size(); i++){
            weight_map_index_type index = get_map_index(x[i], y[i]);
            auto& w = wm[index];
            if(w.hit_count == 0 && w.miss_count == 0){
                w.x = index.first * pos_r;
                w.y = index.second * pos_r;
                w.hit_count += 1;
            }
            w.last_seen_frame = frame_index;
        }
    }

    void MapBackend::markFreeRay(weight_map_index_type origin, weight_map_index_type target, const std::unordered_set<weight_map_index_type, PairHash>& hit_cells){
        int x0 = origin.first;
        int y0 = origin.second;
        int x1 = target.first;
        int y1 = target.second;

        constexpr int kMaxMiss = 20;
        constexpr int kOccupiedThreshold = 5; // hit이 이 이상이면 이미 장애물 → miss 안 찍음

        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while(x0 != x1 || y0 != y1){
            weight_map_index_type current = {x0, y0};
            if(current != origin && hit_cells.find(current) == hit_cells.end()){
                auto it = wm.find(current);
                if(it != wm.end() && it->second.hit_count >= kOccupiedThreshold){
                    // 이미 장애물로 확인된 셀 → ray가 관통할 수 없음 → ray 중단
                    break;
                }
                touchWeightCell(current);
                Weight& w = wm[current];
                if(w.miss_count < kMaxMiss) w.miss_count += 1;
            }

            int err2 = 2 * err;
            if(err2 > -dy){
                err -= dy;
                x0 += sx;
            }
            if(err2 < dx){
                err += dx;
                y0 += sy;
            }
        }
    }

    void MapBackend::updateOccupancyMap(double sensor_x, double sensor_y, const std::vector<double>& x, const std::vector<double>& y){
        std::unordered_set<weight_map_index_type, PairHash> hit_cells;
        hit_cells.reserve(x.size());
        weight_map_index_type origin = get_map_index(sensor_x, sensor_y);

        constexpr int kMaxCount = 20;  // hit/miss 상한 → 최근 관측에 비중

        for(size_t i=0; i<x.size(); i++){
            weight_map_index_type index = get_map_index(x[i], y[i]);
            hit_cells.insert(index);
        }

        // hit 기록 (상한 적용)
        for(const auto& index : hit_cells){
            touchWeightCell(index);
            Weight& w = wm[index];
            if(w.hit_count < kMaxCount) w.hit_count += 1;
            w.last_seen_frame = frame_index;
            markFreeRay(origin, index, hit_cells);
        }
    }

    bool MapBackend::isStaticCell(const Weight& weight) const{
        int total = weight.hit_count + weight.miss_count;
        if(total < 3) return false;  // 최소 3회 이상 관측된 셀만

        double occupied_ratio = static_cast<double>(weight.hit_count) / total;
        return occupied_ratio >= 0.7;
    }

    void MapBackend::getPos(std::vector<double>& x, std::vector<double>& y, bool static_only){
        x.clear();
        y.clear();
        x.reserve(wm.size());
        y.reserve(wm.size());
        
        for(const auto& entry : wm){
            const Weight& weight = entry.second;
            if(static_only && !isStaticCell(weight)) continue;

            double px = weight.x;
            double py = weight.y;

            x.push_back(px);
            y.push_back(py);
        }
    }

    void MapBackend::getAdjacentPos(const RobotBasePose& pose, std::vector<double>& x, std::vector<double>& y, double radius, bool static_only){
        x.clear();
        y.clear();

        double r2 = radius * radius;
        for(const auto& entry : wm){
            const Weight& weight = entry.second;
            if(static_only && !isStaticCell(weight)) continue;

            double dx = weight.x - pose.tx;
            double dy = weight.y - pose.ty;
            if(dx * dx + dy * dy <= r2){
                x.push_back(weight.x);
                y.push_back(weight.y);
            }
        }
    }

    void MapBackend::getAdjacentPosDownsampled(const RobotBasePose& pose, std::vector<double>& x, std::vector<double>& y, double radius, double downsample_res, bool static_only){
        x.clear();
        y.clear();

        // downsample_res 해상도로 1포인트만 유지 (pos_r보다 coarse한 그리드)
        double inv_ds = 1.0 / downsample_res;
        std::unordered_set<weight_map_index_type, PairHash> seen;

        double r2 = radius * radius;
        for(const auto& entry : wm){
            const Weight& weight = entry.second;
            if(static_only && !isStaticCell(weight)) continue;

            double dx = weight.x - pose.tx;
            double dy = weight.y - pose.ty;
            if(dx * dx + dy * dy <= r2){
                int gx = static_cast<int>(std::floor(weight.x * inv_ds));
                int gy = static_cast<int>(std::floor(weight.y * inv_ds));
                if(seen.emplace(gx, gy).second){
                    x.push_back(weight.x);
                    y.push_back(weight.y);
                }
            }
        }
    }

    void MapBackend::clearMap(){
        wm.clear();
    }

    void MapBackend::swapMapData(MapBackend& other){
        wm.swap(other.wm);
        std::swap(frame_index, other.frame_index);
    }

    void MapBackend::rebuildFromSubmaps(
        const std::vector<double>& sensor_xs, const std::vector<double>& sensor_ys,
        const std::vector<std::vector<double>>& point_xs, const std::vector<std::vector<double>>& point_ys)
    {
        wm.clear();
        size_t n = sensor_xs.size();
        if(n == 0) return;

        // 1) Bounding box 계산
        double bmin_x = sensor_xs[0], bmax_x = sensor_xs[0];
        double bmin_y = sensor_ys[0], bmax_y = sensor_ys[0];
        for(size_t s = 0; s < n; s++){
            bmin_x = std::min(bmin_x, sensor_xs[s]);
            bmax_x = std::max(bmax_x, sensor_xs[s]);
            bmin_y = std::min(bmin_y, sensor_ys[s]);
            bmax_y = std::max(bmax_y, sensor_ys[s]);
            for(size_t i = 0; i < point_xs[s].size(); i++){
                bmin_x = std::min(bmin_x, point_xs[s][i]);
                bmax_x = std::max(bmax_x, point_xs[s][i]);
                bmin_y = std::min(bmin_y, point_ys[s][i]);
                bmax_y = std::max(bmax_y, point_ys[s][i]);
            }
        }

        // Grid 범위 (get_map_index와 동일하게 std::ceil 사용)
        int gmin_x = static_cast<int>(std::floor(bmin_x / pos_r)) - 2;
        int gmin_y = static_cast<int>(std::floor(bmin_y / pos_r)) - 2;
        int gmax_x = static_cast<int>(std::ceil(bmax_x / pos_r)) + 2;
        int gmax_y = static_cast<int>(std::ceil(bmax_y / pos_r)) + 2;

        int W = gmax_x - gmin_x + 1;
        int H = gmax_y - gmin_y + 1;
        size_t grid_size = static_cast<size_t>(W) * H;

        // 2) Flat grid 할당 (hash map 대신 배열 → Bresenham O(1) 접근)
        std::vector<uint8_t> hit_grid(grid_size, 0);
        std::vector<uint8_t> miss_grid(grid_size, 0);
        std::vector<uint8_t> is_hit(grid_size, 0);  // 서브맵 단위 hit 마커

        constexpr uint8_t kMaxCount = 20;
        constexpr uint8_t kMaxMiss = 20;
        constexpr uint8_t kOccThresh = 3;

        // 3) 서브맵별 처리
        std::vector<std::pair<int,int>> hit_cells;  // is_hit 클리어용 인덱스 저장

        for(size_t s = 0; s < n; s++){
            int ox = static_cast<int>(std::ceil(sensor_xs[s] / pos_r));
            int oy = static_cast<int>(std::ceil(sensor_ys[s] / pos_r));

            hit_cells.clear();

            // Hit 셀 마킹 (dedup)
            for(size_t i = 0; i < point_xs[s].size(); i++){
                int gx = static_cast<int>(std::ceil(point_xs[s][i] / pos_r));
                int gy = static_cast<int>(std::ceil(point_ys[s][i] / pos_r));
                int lx = gx - gmin_x, ly = gy - gmin_y;
                if(lx < 0 || lx >= W || ly < 0 || ly >= H) continue;
                size_t idx = static_cast<size_t>(ly) * W + lx;
                if(!is_hit[idx]){
                    is_hit[idx] = 1;
                    hit_cells.emplace_back(gx, gy);
                    if(hit_grid[idx] < kMaxCount) hit_grid[idx]++;
                }
            }

            // Bresenham ray tracing (센서 → hit 셀, free 셀에 miss 기록)
            for(const auto& [hx, hy] : hit_cells){
                int x0 = ox, y0 = oy;
                int dx = std::abs(hx - x0), dy = std::abs(hy - y0);
                int sx = (x0 < hx) ? 1 : -1, sy = (y0 < hy) ? 1 : -1;
                int err = dx - dy;

                while(x0 != hx || y0 != hy){
                    if(x0 != ox || y0 != oy){
                        int lx = x0 - gmin_x, ly = y0 - gmin_y;
                        if(lx >= 0 && lx < W && ly >= 0 && ly < H){
                            size_t ci = static_cast<size_t>(ly) * W + lx;
                            if(is_hit[ci]){
                                // hit 셀은 건너뜀
                            } else if(hit_grid[ci] >= kOccThresh){
                                break;  // 이미 장애물 → ray 중단
                            } else {
                                if(miss_grid[ci] < kMaxMiss) miss_grid[ci]++;
                            }
                        }
                    }
                    int err2 = 2 * err;
                    if(err2 > -dy){ err -= dy; x0 += sx; }
                    if(err2 < dx){ err += dx; y0 += sy; }
                }
            }

            // is_hit 클리어 (다음 서브맵을 위해)
            for(const auto& [gx, gy] : hit_cells){
                int lx = gx - gmin_x, ly = gy - gmin_y;
                is_hit[static_cast<size_t>(ly) * W + lx] = 0;
            }
        }

        // 4) Flat grid → hash map 변환 (1회만)
        for(int ly = 0; ly < H; ly++){
            for(int lx = 0; lx < W; lx++){
                size_t idx = static_cast<size_t>(ly) * W + lx;
                if(hit_grid[idx] > 0 || miss_grid[idx] > 0){
                    int gx = lx + gmin_x, gy = ly + gmin_y;
                    Weight w;
                    w.x = gx * pos_r;
                    w.y = gy * pos_r;
                    w.hit_count = hit_grid[idx];
                    w.miss_count = miss_grid[idx];
                    wm[{gx, gy}] = w;
                }
            }
        }
    }

    void MapBackend::incrementFrameIndex(){
        frame_index++;
    }

    void MapBackend::getOccupancyGridData(std::vector<int8_t>& data, int& width, int& height,
                                          double& origin_x, double& origin_y) const
    {
        if(wm.empty()){
            data.clear();
            width = height = 0;
            origin_x = origin_y = 0;
            return;
        }

        // 셀 인덱스 범위 계산
        auto it = wm.begin();
        int min_gx = it->first.first, max_gx = min_gx;
        int min_gy = it->first.second, max_gy = min_gy;
        for(; it != wm.end(); ++it){
            min_gx = std::min(min_gx, it->first.first);
            max_gx = std::max(max_gx, it->first.first);
            min_gy = std::min(min_gy, it->first.second);
            max_gy = std::max(max_gy, it->first.second);
        }

        width = max_gx - min_gx + 1;
        height = max_gy - min_gy + 1;
        origin_x = min_gx * pos_r;
        origin_y = min_gy * pos_r;

        data.assign(static_cast<size_t>(width) * height, -1);

        for(const auto& entry : wm){
            int lx = entry.first.first - min_gx;
            int ly = entry.first.second - min_gy;
            size_t idx = static_cast<size_t>(ly) * width + lx;

            const Weight& w = entry.second;
            int total = w.hit_count + w.miss_count;
            if(total == 0) continue;

            double occ_ratio = static_cast<double>(w.hit_count) / total;
            if(occ_ratio >= 0.80 && total >= 5){
                data[idx] = 100;  // occupied
            } else {
                data[idx] = 0;    // free
            }
        }
    }
}