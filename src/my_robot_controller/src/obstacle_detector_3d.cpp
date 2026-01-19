#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ObstacleDetector3D : public rclcpp::Node {
public:
    ObstacleDetector3D() : Node("obstacle_detector_3d") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&ObstacleDetector3D::pc_callback, this, std::placeholders::_1));
    }

private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        float min_dist = 100.0;

        // Iterate through 3D points
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // Check points only in front of the robot (X > 0)
            if (*iter_x > 0 && std::abs(*iter_y) < 0.2) {
                float dist = std::sqrt((*iter_x)*(*iter_x) + (*iter_y)*(*iter_y));
                if (dist < min_dist) min_dist = dist;
            }
        }

        if (min_dist < 0.7) {
            RCLCPP_WARN(this->get_logger(), "3D OBSTACLE DETECTED! DIST: %.2f", min_dist);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetector3D>());
    rclcpp::shutdown();
    return 0;
}
