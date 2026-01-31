#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class Costmap2DFrom3D : public rclcpp::Node {
public:
    Costmap2DFrom3D() : Node("costmap_2d_from_3d") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/global_map_3d", rclcpp::QoS(1).transient_local(), std::bind(&Costmap2DFrom3D::process_cloud, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap_2d", rclcpp::QoS(1).transient_local());
    }

private:
    void process_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header = msg->header;
        grid.info.resolution = 0.05; // 5cm
        grid.info.width = 400;       // 20 meters wide
        grid.info.height = 400;
        grid.info.origin.position.x = -10.0; // Center the map
        grid.info.origin.position.y = -10.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(grid.info.width * grid.info.height, 0);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (*iter_z < 0.1 || *iter_z > 1.5) continue; // Height filter

            int mx = (*iter_x - grid.info.origin.position.x) / grid.info.resolution;
            int my = (*iter_y - grid.info.origin.position.y) / grid.info.resolution;

            if (mx >= 0 && mx < (int)grid.info.width && my >= 0 && my < (int)grid.info.height) {
                grid.data[my * grid.info.width + mx] = 100; // Mark occupied
            }
        }
        publisher_->publish(grid);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Costmap2DFrom3D>());
    rclcpp::shutdown();
    return 0;
}
