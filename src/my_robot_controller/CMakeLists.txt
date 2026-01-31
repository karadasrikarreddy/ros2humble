cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED) # Needed for map saver service
find_package(std_srvs REQUIRED)

include_directories(include)

# ==========================================
# 1. MAPPING & LOADING NODES
# ==========================================

# Map Accumulator (Creates 3D maps)
add_executable(map_accumulator_3d src/map_accumulator_3d.cpp)
ament_target_dependencies(map_accumulator_3d 
  rclcpp sensor_msgs pcl_conversions pcl_ros tf2_ros tf2_sensor_msgs std_srvs
)

# Dynamic Map Manager (Loads PCD & publishes to /global_map_3d)
add_executable(dynamic_3d_map_manager src/dynamic_3d_map_manager.cpp)
ament_target_dependencies(dynamic_3d_map_manager rclcpp sensor_msgs pcl_conversions pcl_ros std_srvs)

# Simple Map Loader (Alternative to dynamic manager)
add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node rclcpp sensor_msgs pcl_conversions pcl_ros)

# ==========================================
# 2. NAVIGATION PIPELINE NODES
# ==========================================

# Costmap Converter (3D -> 2D)
add_executable(costmap_2d_from_3d src/costmap_2d_from_3d.cpp)
ament_target_dependencies(costmap_2d_from_3d rclcpp sensor_msgs nav_msgs pcl_conversions pcl_ros)

# A* Planner (Global Planning)
add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Path Follower (Drives the robot)
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# ==========================================
# 3. SAFETY & UTILITY NODES
# ==========================================

# Obstacle Detector (Stops robot if too close)
add_executable(obstacle_detector_3d src/obstacle_detector_3d.cpp)
ament_target_dependencies(obstacle_detector_3d rclcpp sensor_msgs geometry_msgs)

# Lidar Listener (Simple check)
add_executable(lidar_listener src/lidar_listener.cpp)
ament_target_dependencies(lidar_listener rclcpp sensor_msgs)

# ==========================================
# 4. INSTALLATION
# ==========================================

install(TARGETS
  map_accumulator_3d
  dynamic_3d_map_manager
  map_loader_node
  costmap_2d_from_3d
  a_star_planner
  path_follower
  obstacle_detector_3d
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
