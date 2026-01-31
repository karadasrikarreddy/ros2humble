#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <queue>

struct NodeGrid { int x, y; float cost; float dist; bool operator>(const NodeGrid& other) const { return (cost + dist) > (other.cost + other.dist); }};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("a_star_planner") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap_2d", 1, 
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { costmap_ = *msg; map_received_ = true; });

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, 
            std::bind(&AStarPlanner::plan_path, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);
    }

private:
    void plan_path(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        if (!map_received_) { RCLCPP_WARN(get_logger(), "No Map!"); return; }

        // Get Robot Position from TF
        geometry_msgs::msg::TransformStamped t;
        try { t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero); }
        catch (...) { RCLCPP_ERROR(get_logger(), "TF Error"); return; }

        int start_x = (t.transform.translation.x - costmap_.info.origin.position.x) / costmap_.info.resolution;
        int start_y = (t.transform.translation.y - costmap_.info.origin.position.y) / costmap_.info.resolution;
        int goal_x = (goal->pose.position.x - costmap_.info.origin.position.x) / costmap_.info.resolution;
        int goal_y = (goal->pose.position.y - costmap_.info.origin.position.y) / costmap_.info.resolution;

        // Simple A* (Optimized for brevity)
        std::priority_queue<NodeGrid, std::vector<NodeGrid>, std::greater<NodeGrid>> open_list;
        open_list.push({start_x, start_y, 0.0, 0.0});
        std::map<int, int> came_from;
        std::map<int, float> cost_so_far;
        cost_so_far[start_y * costmap_.info.width + start_x] = 0;

        while (!open_list.empty()) {
            NodeGrid current = open_list.top(); open_list.pop();
            if (current.x == goal_x && current.y == goal_y) break;

            int dx[] = {1, -1, 0, 0}; int dy[] = {0, 0, 1, -1};
            for (int i=0; i<4; i++) {
                int nx = current.x + dx[i], ny = current.y + dy[i];
                int idx = ny * costmap_.info.width + nx;
                if (nx < 0 || ny < 0 || nx >= (int)costmap_.info.width || ny >= (int)costmap_.info.height) continue;
                if (costmap_.data[idx] > 50) continue; // Obstacle check

                float new_cost = cost_so_far[current.y * costmap_.info.width + current.x] + 1.0;
                if (cost_so_far.find(idx) == cost_so_far.end() || new_cost < cost_so_far[idx]) {
                    cost_so_far[idx] = new_cost;
                    float priority = new_cost + std::hypot(goal_x - nx, goal_y - ny);
                    open_list.push({nx, ny, new_cost, priority});
                    came_from[idx] = current.y * costmap_.info.width + current.x;
                }
            }
        }

        // Reconstruct Path
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = now();
        int curr_idx = goal_y * costmap_.info.width + goal_x;
        if (came_from.find(curr_idx) == came_from.end()) { RCLCPP_WARN(get_logger(), "No Path Found"); return; }

        while (curr_idx != static_cast<int>(start_y * costmap_.info.width + start_x)) {
            geometry_msgs::msg::PoseStamped p;
            p.pose.position.x = (curr_idx % costmap_.info.width) * costmap_.info.resolution + costmap_.info.origin.position.x;
            p.pose.position.y = (curr_idx / costmap_.info.width) * costmap_.info.resolution + costmap_.info.origin.position.y;
            path_msg.poses.push_back(p);
            curr_idx = came_from[curr_idx];
        }
        std::reverse(path_msg.poses.begin(), path_msg.poses.end());
        path_pub_->publish(path_msg);
        RCLCPP_INFO(get_logger(), "Path Published!");
    }

    nav_msgs::msg::OccupancyGrid costmap_;
    bool map_received_ = false;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}
