#ifndef __STANDALONE_GLOBAL_PLANNER_H__
#define __STANDALONE_GLOBAL_PLANNER_H__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class StandaloneGlobalPlanner : public rclcpp::Node{
private:
    struct QueueNode
    {
        int index;
        double f_score;
    };
    struct QueueCompare
    {
        bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
        {
            return lhs.f_score > rhs.f_score;
        }
    };

    std::string map_topic_;
    std::string pose_topic_;
    std::string goal_topic_;
    std::string plan_topic_;
    int obstacle_threshold_ = 50;
    bool allow_unknown_ = true;
    double robot_radius_ = 0.25;
    bool use_diagonal_ = true;
    double max_snap_distance_ = 0.5;
    bool map_transient_local_ = false;

    bool has_map_ = false;
    bool has_pose_ = false;
    bool has_goal_ = false;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::vector<uint8_t> inflated_grid_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void makeAndPublishPlan();
    void buildInflatedGrid();
    std::vector<int> runAStar(const int start_index, const int goal_index) const;
    std::vector<std::pair<int, int>> neighbors(const int x, const int y) const;
    std::vector<int> reconstructPath(const std::vector<int> & came_from, const int start_index, const int goal_index) const;
    nav_msgs::msg::Path buildPathMessage(const std::vector<int> & path_indices) const;
    bool findNearestFreeCell(const int source_index, const int max_radius_cells, int & free_index) const;
    bool worldToMap(const geometry_msgs::msg::Point & point, int & mx, int & my) const;
    geometry_msgs::msg::Point mapToWorld(const int mx, const int my) const;
    double heuristic(const int from_index, const int to_index) const;
    double traversalCost(const int index) const;
    bool isObstacleValue(const int8_t value) const;
    bool isFree(const int index) const;
    inline bool isFree(const int x, const int y) const { return inBounds(x, y) && isFree(toIndex(x, y)); }
    bool inBounds(const int x, const int y) const;
    inline int toIndex(const int x, const int y) const { return y * static_cast<int>(map_->info.width) + x; }
    inline std::string mapFrame() const { return map_->header.frame_id.empty() ? "map" : map_->header.frame_id; }
    void warnIfFrameMismatch(const std::string & frame_id, const char * source_name) const;
    geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) const;
    double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
public:
  StandaloneGlobalPlanner();
};

#endif