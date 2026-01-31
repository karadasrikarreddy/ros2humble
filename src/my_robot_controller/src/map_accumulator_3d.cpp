#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h> // <--- NEW HEADER

class MapAccumulator3D : public rclcpp::Node {
public:
    MapAccumulator3D() : Node("map_accumulator_3d") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/points", 10, std::bind(&MapAccumulator3D::pc_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_3d", 10);
        
        save_service_ = this->create_service<std_srvs::srv::Trigger>(
            "save_map_3d", std::bind(&MapAccumulator3D::save_map_callback, this, std::placeholders::_1, std::placeholders::_2));

        global_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        RCLCPP_INFO(this->get_logger(), "✅ CLEAN 3D Mapper Ready (Ground Filter Active).");
    }

private:
    void save_map_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (global_cloud_->empty()) {
            response->success = false;
            response->message = "Map is empty!";
            return;
        }

        std::string filename = "/root/ros2_ws/vs/src/my_robot_description/maps/my_3d_map.pcd";
        try {
            pcl::io::savePCDFileASCII(filename, *global_cloud_);
            response->success = true;
            response->message = "Map saved to: " + filename;
            RCLCPP_INFO(this->get_logger(), "💾 Saved FILTERED Map: %lu points", global_cloud_->size());
        } catch (const std::exception &e) {
            response->success = false;
            response->message = "Failed to save.";
        }
    }

    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        sensor_msgs::msg::PointCloud2 cloud_out;
        try {
            if (!tf_buffer_->canTransform("odom", msg->header.frame_id, tf2::TimePointZero)) return;
            transform = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*msg, cloud_out, transform);
        } catch (tf2::TransformException &ex) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_out, *current_cloud);

        // --- NEW: Ground Removal Filter ---
        // Before adding this data to the map, remove the floor.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(current_cloud);
        pass.setFilterFieldName("z");
        
        // TUNE THIS: 0.05 means "remove everything below 5cm height"
        pass.setFilterLimits(0.05, 10.0); 
        pass.filter(*cloud_no_ground);
        // ----------------------------------

        *global_cloud_ += *cloud_no_ground;

        // Downsample to keep it fast
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(global_cloud_);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);
        global_cloud_ = filtered_cloud;

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*global_cloud_, output_msg);
        output_msg.header.frame_id = "odom";
        output_msg.header.stamp = this->get_clock()->now();
        publisher_->publish(output_msg);
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapAccumulator3D>());
    rclcpp::shutdown();
    return 0;
}
