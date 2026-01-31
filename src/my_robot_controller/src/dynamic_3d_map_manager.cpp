#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <std_srvs/srv/trigger.hpp> // Standard "Do it!" service

class Dynamic3DMapManager : public rclcpp::Node {
public:
    Dynamic3DMapManager() : Node("dynamic_3d_map_manager") {
        // Parameter for the file path
        this->declare_parameter<std::string>("pcd_file_path", "");

        // Publisher (Latched / Transient Local)
        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        qos_profile.reliable();
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_map_3d", qos_profile);

        // Service Server: /load_map
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "load_map", std::bind(&Dynamic3DMapManager::load_map_service, this, std::placeholders::_1, std::placeholders::_2));

        // Attempt to load immediately on startup
        load_and_publish();
    }

private:
    // The core logic function
    bool load_and_publish() {
        std::string pcd_path;
        this->get_parameter("pcd_file_path", pcd_path);

        if (pcd_path.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠️ No PCD path parameter set. Waiting for service call...");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "📂 Loading map from: %s", pcd_path.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "❌ FAILED to open file: %s", pcd_path.c_str());
            return false;
        }

        // --- FILTERING (Ground Removal) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, 5.0); // Remove floor (< 5cm) and ceiling (> 5m)
        pass.filter(*cloud_filtered);
        // ----------------------------------

        // Convert to ROS Message
        sensor_msgs::msg::PointCloud2 map_msg;
        pcl::toROSMsg(*cloud_filtered, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(map_msg);
        RCLCPP_INFO(this->get_logger(), "✅ Map Published! Points: %lu (Raw: %lu)", cloud_filtered->size(), cloud->size());
        return true;
    }

    // Service Callback
    void load_map_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request; // Unused
        if (load_and_publish()) {
            response->success = true;
            response->message = "Map loaded and published successfully.";
        } else {
            response->success = false;
            response->message = "Failed to load map. Check logs/path.";
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Dynamic3DMapManager>());
    rclcpp::shutdown();
    return 0;
}
