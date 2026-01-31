#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm> 

struct AStarNode {
    int x, y;
    double g_cost, h_cost;
    std::shared_ptr<AStarNode> parent; 

    double f_cost() const { return g_cost + h_cost; }
    
    // For sorting in priority queue (lowest cost first)
    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("a_star_planner") {
        // Subscribers
        sub_costmap_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap_2d", rclcpp::QoS(1).transient_local(), 
            std::bind(&AStarPlanner::costmap_callback, this, std::placeholders::_1));

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, 
            std::bind(&AStarPlanner::goal_callback, this, std::placeholders::_1));

        // Publisher
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

        // TF Listener (to find robot start position)
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	
	this->declare_parameter("obstacle_threshold", 50);
        RCLCPP_INFO(this->get_logger(), "✅ A* Planner Ready. Waiting for Map & Goal...");
    }

private:
    void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        costmap_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Map Updated in Planner.");
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (costmap_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan: Costmap is empty!");
            return;
        }

        // 1. Get Start Position (Robot's current location)
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get robot position: %s", ex.what());
            return;
        }

        int start_x, start_y, goal_x, goal_y;
        if (!worldToMap(t.transform.translation.x, t.transform.translation.y, start_x, start_y) ||
            !worldToMap(msg->pose.position.x, msg->pose.position.y, goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "Start or Goal is out of map bounds!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planning from (%d, %d) to (%d, %d)...", start_x, start_y, goal_x, goal_y);
        
        // 2. Run A* Algorithm
        std::vector<std::pair<int, int>> path_points;
        if (run_a_star(start_x, start_y, goal_x, goal_y, path_points)) {
            publish_path(path_points);
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ No path found (Goal might be in a wall)");
        }
    }

    // RENAMED ALL 'Node' references to 'AStarNode'
    bool run_a_star(int start_x, int start_y, int goal_x, int goal_y, std::vector<std::pair<int, int>>& out_path) {
        // Priority Queue for Open Set
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
        
        // Visited Set (keep track of cheapest cost to reach a cell)
        std::vector<double> visited(costmap_.info.width * costmap_.info.height, std::numeric_limits<double>::infinity());

        // Add Start Node
        AStarNode start_node;
        start_node.x = start_x;
        start_node.y = start_y;
        start_node.g_cost = 0.0;
        start_node.h_cost = std::hypot(goal_x - start_x, goal_y - start_y);
        start_node.parent = nullptr;
        
        open_list.push(start_node);
        visited[start_y * costmap_.info.width + start_x] = 0.0;

        while (!open_list.empty()) {
            AStarNode current = open_list.top();
            open_list.pop();

            // Check if goal reached
            if (current.x == goal_x && current.y == goal_y) {
                // Reconstruct path
                while (current.parent != nullptr) {
                    out_path.push_back({current.x, current.y});
                    current = *current.parent;
                }
                out_path.push_back({start_x, start_y}); // Add start
                std::reverse(out_path.begin(), out_path.end());
                return true;
            }

            // Check Neighbors (8-connected)
            int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
            int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};
            double costs[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414}; // Diagonals cost more

            for (int i = 0; i < 8; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                // Bounds Check
                if (nx < 0 || ny < 0 || nx >= (int)costmap_.info.width || ny >= (int)costmap_.info.height) continue;

                // Collision Check (Map value > 50 is an obstacle)
                int index = ny * costmap_.info.width + nx;
                if (costmap_.data[index] > 50) //continue;
			{
			       // Read the parameter dynamically
				int threshold = this->get_parameter("obstacle_threshold").as_int();

				// Collision Check
				if (costmap_.data[index] > threshold) continue;
			}

                double new_g_cost = current.g_cost + costs[i];

                if (new_g_cost < visited[index]) {
                    visited[index] = new_g_cost;
                    
                    AStarNode neighbor;
                    neighbor.x = nx;
                    neighbor.y = ny;
                    neighbor.g_cost = new_g_cost;
                    neighbor.h_cost = std::hypot(goal_x - nx, goal_y - ny);
                    // Use std::make_shared with the new struct name
                    neighbor.parent = std::make_shared<AStarNode>(current);
                    
                    open_list.push(neighbor);
                }
            }
        }
        return false;
    }

    void publish_path(const std::vector<std::pair<int, int>>& path_points) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->get_clock()->now();

        for (const auto& p : path_points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.first * costmap_.info.resolution + costmap_.info.origin.position.x;
            pose.pose.position.y = p.second * costmap_.info.resolution + costmap_.info.origin.position.y;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        pub_path_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "✅ Path Published! Length: %lu steps", path_points.size());
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) {
        if (costmap_.info.resolution == 0) return false;
        mx = (int)((wx - costmap_.info.origin.position.x) / costmap_.info.resolution);
        my = (int)((wy - costmap_.info.origin.position.y) / costmap_.info.resolution);
        return true;
    }

    nav_msgs::msg::OccupancyGrid costmap_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_costmap_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}
