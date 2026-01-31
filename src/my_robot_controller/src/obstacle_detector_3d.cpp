#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <atomic>

class ObstacleDetector3D : public rclcpp::Node {
public:
    ObstacleDetector3D() : Node("obstacle_detector_3d") {
        // --- ROBOT DIMENSIONS (The "Box") ---
        this->declare_parameter("robot_width_half", 0.25);  // 0.2m actual + 0.05m buffer
        this->declare_parameter("robot_length_front", 0.35); // 0.3m actual + 0.05m buffer
        this->declare_parameter("robot_length_back", -0.35); // Back bumper
        this->declare_parameter("reaction_time", 1.0);       // Look ahead time

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&ObstacleDetector3D::lidar_callback, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10, std::bind(&ObstacleDetector3D::cmd_callback, this, std::placeholders::_1));

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "🛡️ POLYGON SAFETY GUARD ACTIVE.");
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Store the closest object distance IN THE FRONT (X-axis)
        // We will assume 'width' is safe if the path planner did its job,
        // but we double-check the width here to be sure.
        
        float closest_x = 100.0;
        double width_limit = this->get_parameter("robot_width_half").as_double();

        for (const auto& pt : cloud.points) {
            // 1. Z-Filter (Floor/Ceiling)
            if (pt.z < 0.02 || pt.z > 1.0) continue;

            // 2. SELF FILTER (Inner Box)
            // Ignore points literally inside the robot chassis (0.6 x 0.4)
            if (pt.x > -0.3 && pt.x < 0.3 && pt.y > -0.2 && pt.y < 0.2) continue;

            // 3. SAFETY TUNNEL CHECK
            // Is this point within our "driving lane" width?
            if (std::abs(pt.y) < width_limit) {
                // Yes, it's in our lane. How close is it in front of us?
                if (pt.x > 0 && pt.x < closest_x) {
                    closest_x = pt.x;
                }
            }
        }
        min_forward_dist_.store(closest_x);
    }

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Twist safe_cmd = *msg;
        
        double front_base = this->get_parameter("robot_length_front").as_double();
        double reaction_time = this->get_parameter("reaction_time").as_double();

        // --- DYNAMIC BOX CALCULATION ---
        // How far do we need to stop given current request speed?
        double speed = std::abs(safe_cmd.linear.x);
        
        // The box stretches: Physical Front + (Speed * Time)
        double stop_threshold = front_base + (speed * reaction_time);
        
        // Slow down zone starts 0.5m before the stop zone
        double slow_threshold = stop_threshold + 0.5;

        // Get actual obstacle distance
        float obs_x = min_forward_dist_.load();

        if (obs_x < stop_threshold) {
            // OBSTACLE INSIDE STOP BOX -> HARD STOP
            safe_cmd.linear.x = 0.0;
            
            // Allow turning in place (to escape), but stop moving forward
            // (Unless the obstacle is super close, then freeze everything)
            if (obs_x < front_base + 0.1) safe_cmd.angular.z = 0.0;

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🛑 BOX STOP! Obj: %.2fm < Limit: %.2fm", obs_x, stop_threshold);
        }
        else if (obs_x < slow_threshold) {
            // OBSTACLE IN SLOW ZONE -> SCALE SPEED
            double factor = (obs_x - stop_threshold) / (slow_threshold - stop_threshold);
            if (factor < 0.1) factor = 0.1; // Don't stall
            safe_cmd.linear.x *= factor;
        }

        pub_cmd_->publish(safe_cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    
    std::atomic<float> min_forward_dist_{100.0f}; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector3D>());
    rclcpp::shutdown();
    return 0;
}
