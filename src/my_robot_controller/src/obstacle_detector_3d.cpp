#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/twist.hpp"

class EmergencyBrake3D : public rclcpp::Node {
public:
    EmergencyBrake3D() : Node("emergency_brake_3d") {
        // Subscriber for Lidar
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&EmergencyBrake3D::pc_callback, this, std::placeholders::_1));
        
        // Publisher for the stop command
        stop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Safety System Active: Monitoring 3D Environment...");
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        float min_dist = 100.0;
        bool collision_imminent = false;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            // Focus on a "Safety Box" in front of the robot: 
            // X (Forward) between 0 and 0.8m, Y (Side) within 0.3m
            if (*iter_x > 0.05 && *iter_x < 0.8 && std::abs(*iter_y) < 0.3) {
                collision_imminent = true;
                break; 
            }
        }

        if (collision_imminent) {
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY BRAKE TRIGGERED!");
            auto stop_msg = geometry_msgs::msg::Twist();
            stop_msg.linear.x = 0.0;
            stop_msg.angular.z = 0.0;
            stop_pub_->publish(stop_msg);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmergencyBrake3D>());
    rclcpp::shutdown();
    return 0;
}
