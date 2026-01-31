#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <limits>

class ObstacleDetector3D : public rclcpp::Node
{
public:
  ObstacleDetector3D()
  : Node("obstacle_detector_3d")
  {
    /* ---------------- Parameters ---------------- */
    declare_parameter("max_speed", 0.8);
    declare_parameter("stop_distance", 0.5);
    declare_parameter("caution_distance", 2.0);
    declare_parameter("robot_radius", 0.5);

    max_speed_     = get_parameter("max_speed").as_double();
    stop_dist_     = get_parameter("stop_distance").as_double();
    caution_dist_  = get_parameter("caution_distance").as_double();
    robot_radius_  = get_parameter("robot_radius").as_double();

    /* ---------------- Subscribers ---------------- */
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetector3D::cloudCallback, this, std::placeholders::_1));

    cmd_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw", 10,
      std::bind(&ObstacleDetector3D::cmdCallback, this, std::placeholders::_1));

    /* ---------------- Publisher ---------------- */
    cmd_safe_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    RCLCPP_INFO(get_logger(), "Obstacle Detector 3D started (PCL-free)");
  }

private:
  /* ================= CALLBACKS ================= */

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    min_obstacle_dist_ = std::numeric_limits<float>::max();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      const float x = *iter_x;
      const float y = *iter_y;

      if (!std::isfinite(x) || !std::isfinite(y))
        continue;

      // Distance in robot ground plane
      const float dist = std::sqrt(x * x + y * y);

      // Ignore points inside robot footprint
      if (dist <= robot_radius_)
        continue;

      min_obstacle_dist_ = std::min(min_obstacle_dist_, dist);
    }
  }

  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::Twist safe_cmd = *msg;

    const float speed_scale = computeSpeedScale(min_obstacle_dist_);

    safe_cmd.linear.x  = max_speed_ * speed_scale;
    safe_cmd.linear.y  = 0.0;                // diff-drive safety
    safe_cmd.angular.z = msg->angular.z;

    // Emergency stop
    if (min_obstacle_dist_ <= stop_dist_)
    {
      safe_cmd.linear.x  = 0.0;
      safe_cmd.angular.z = 0.0;
    }

    cmd_safe_pub_->publish(safe_cmd);
  }

  /* ================= LOGIC ================= */

  float computeSpeedScale(float obstacle_dist) const
  {
    if (obstacle_dist <= stop_dist_)
      return 0.0;

    if (obstacle_dist >= caution_dist_)
      return 1.0;

    return (obstacle_dist - stop_dist_) /
           (caution_dist_ - stop_dist_);
  }

  /* ================= MEMBERS ================= */

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_safe_pub_;

  float max_speed_;
  float stop_dist_;
  float caution_dist_;
  float robot_radius_;

  float min_obstacle_dist_ = std::numeric_limits<float>::max();
};

/* ================= MAIN ================= */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector3D>());
  rclcpp::shutdown();
  return 0;
}
