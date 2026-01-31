#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PathFollower : public rclcpp::Node {
public:
    PathFollower() : Node("path_follower") {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/planned_path", 1, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = *msg; });
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PathFollower::control_loop, this));
    }

private:
    void control_loop() {
        if (current_path_.poses.empty()) return;

        // Get Robot Pose
        geometry_msgs::msg::TransformStamped t;
        try { t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero); }
        catch (...) { return; }

        // Find Lookahead Point (First point > 0.5m away)
        geometry_msgs::msg::PoseStamped target;
        bool found = false;
        double robot_x = t.transform.translation.x;
        double robot_y = t.transform.translation.y;

        for (auto &p : current_path_.poses) {
            double dist = std::hypot(p.pose.position.x - robot_x, p.pose.position.y - robot_y);
            if (dist > 0.5) { target = p; found = true; break; }
        }

        geometry_msgs::msg::Twist cmd;
        if (found) {
            double dx = target.pose.position.x - robot_x;
            double dy = target.pose.position.y - robot_y;
            double angle_to_target = atan2(dy, dx);
            
            // Convert quaternion to yaw
            tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            cmd.angular.z = 2.0 * (angle_to_target - yaw); // Proportional steering
            cmd.linear.x = 0.5; // Constant speed
        }
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    nav_msgs::msg::Path current_path_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}
