#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    RobotDriver() : Node("robot_driver") {
        // Create a publisher to the /cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Timer to call the loop every 100ms
        timer_ = this->create_wall_timer(100ms, std::bind(&RobotDriver::send_command, this));
    }

private:
    void send_command() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.5;  // Move forward at 0.5 m/s
        message.angular.z = 0.3; // Turn at 0.3 rad/s
        RCLCPP_INFO(this->get_logger(), "Sending C++ Command: Linear=0.5, Angular=0.3");
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}
