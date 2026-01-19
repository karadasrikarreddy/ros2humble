cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- DRIVE NODE ---
# 1. Create the executable first
add_executable(drive_node src/drive_node.cpp)
# 2. Then link the dependencies to THAT target
ament_target_dependencies(drive_node rclcpp geometry_msgs)

# --- LIDAR LISTENER (3D) ---
# 1. Create the executable
add_executable(lidar_listener src/obstacle_detector_3d.cpp)
# 2. Link the dependencies
ament_target_dependencies(lidar_listener rclcpp sensor_msgs)

install(TARGETS
  drive_node
  lidar_listener
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
