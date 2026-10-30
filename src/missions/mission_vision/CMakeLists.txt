cmake_minimum_required(VERSION 3.8)
project(mission_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 1. traffic_light_node (1.3m height Red/Green LED state detector)
add_executable(traffic_light_node src/traffic_light_node.cpp)
ament_target_dependencies(traffic_light_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 2. parking_marker_node (10x10cm QR / 'P' sign detector)
add_executable(parking_marker_node src/parking_marker_node.cpp)
ament_target_dependencies(parking_marker_node
  rclcpp
  std_msgs
  sensor_msgs
)

# 3. cone_detector_node (23cm traffic cone RGB-D + LiDAR 3D fusion detector)
add_executable(cone_detector_node src/cone_detector_node.cpp)
ament_target_dependencies(cone_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  traffic_light_node
  parking_marker_node
  cone_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
