cmake_minimum_required(VERSION 3.8)
project(go2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

include_directories(include)

# 1. odom_tf_node
add_executable(odom_tf_node src/odom_tf_node.cpp)
ament_target_dependencies(odom_tf_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

# 2. imu_bridge_node
add_executable(imu_bridge_node src/imu_bridge_node.cpp)
ament_target_dependencies(imu_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  unitree_go
)

# 3. cloud_bridge_node (3D PointCloud + Hill-Aware Terrain Scan Slicer)
add_executable(cloud_bridge_node src/cloud_bridge_node.cpp)
ament_target_dependencies(cloud_bridge_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# 4. cmd_vel_to_sport_node
add_executable(cmd_vel_to_sport_node src/cmd_vel_to_sport_node.cpp)
ament_target_dependencies(cmd_vel_to_sport_node
  rclcpp
  geometry_msgs
  unitree_api
)

install(TARGETS
  odom_tf_node
  imu_bridge_node
  cloud_bridge_node
  cmd_vel_to_sport_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
