cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# ... add to ament_target_dependencies for map_accumulator_3d ...
# 2. Add the Map Accumulator executable
add_executable(map_accumulator_3d src/map_accumulator_3d.cpp)
ament_target_dependencies(map_accumulator_3d 
  rclcpp 
  sensor_msgs 
  pcl_conversions 
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  std_srvs
)

# 3. Keep your previous nodes here too
add_executable(lidar_listener src/obstacle_detector_3d.cpp)
ament_target_dependencies(lidar_listener rclcpp sensor_msgs geometry_msgs)

# 4. INSTALLation - This is what fixes your "not found" error
install(TARGETS
  map_accumulator_3d
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
