#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <atomic> // Ensure this is included

class ObstacleDetector3D : public rclcpp::Node {
public:
    ObstacleDetector3D() : Node("obstacle_detector_3d") {
        // --- CONFIGURATION ---
        this->declare_parameter("robot_radius", 0.35); 
        this->declare_parameter("reaction_time", 1.5); 
        this->declare_parameter("safe_buffer", 0.10);  

        // Subscribers
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&ObstacleDetector3D::lidar_callback, this, std::placeholders::_1));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10, std::bind(&ObstacleDetector3D::cmd_callback, this, std::placeholders::_1));

        // Publisher
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "🛡️ DYNAMIC SAFETY GUARD ACTIVE.");
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        float min_dist = 100.0;

        for (const auto& pt : cloud.points) {
            // 1. Height Filter (Ignore floor and ceiling)
            if (pt.z < 0.05 || pt.z > 1.0) continue;

            // 2. Distance Calculation
            float dist = std::hypot(pt.x, pt.y);

            // 3. SELF-FILTER (Crucial Fix)
            // Ignore points inside the robot chassis (0.4m radius)
            // This prevents the robot from seeing its own camera/wheels as obstacles.
            if (dist < 0.4) continue;

            // 4. Find closest VALID obstacle
            if (dist < min_dist) min_dist = dist;
        }
        current_obstacle_dist_ = min_dist;
    }
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        geometry_msgs::msg::Twist safe_cmd = *msg;
        
        double robot_radius = this->get_parameter("robot_radius").as_double();
        double reaction_time = this->get_parameter("reaction_time").as_double();
        double buffer = this->get_parameter("safe_buffer").as_double();

        // --- DYNAMIC SAFETY CALCULATION ---
        double requested_speed = std::abs(safe_cmd.linear.x);
        double required_stop_dist = robot_radius + (requested_speed * reaction_time) + buffer;
        double slow_down_dist = required_stop_dist * 1.5; 

        // Get the current distance safely
        float obs_dist = current_obstacle_dist_.load(); 

        if (obs_dist < required_stop_dist) {
            // CRITICAL: STOP
            safe_cmd.linear.x = 0.0;
            safe_cmd.angular.z = 0.0; 
            
            // FIX: Use .load() or the local variable 'obs_dist' here
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "🛑 EMERGENCY STOP! Obj: %.2fm < Req: %.2fm", obs_dist, required_stop_dist);
        }
        else if (obs_dist < slow_down_dist) {
            // WARNING: SLOW DOWN
            double factor = (obs_dist - required_stop_dist) / (slow_down_dist - required_stop_dist);
            safe_cmd.linear.x *= factor;
            if (factor < 0.5) safe_cmd.angular.z *= factor;
        }

        pub_cmd_->publish(safe_cmd);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    
    std::atomic<float> current_obstacle_dist_{100.0f}; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector3D>());
    rclcpp::shutdown();
    return 0;
}
