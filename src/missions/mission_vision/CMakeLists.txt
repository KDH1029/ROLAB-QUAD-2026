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
find_package(rclcpp_lifecycle REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Cone localisation & tracking (YOLO boxes + LiDAR/depth -> world frame)
add_executable(cone_tracker_node src/cone_tracker_node.cpp)
target_include_directories(cone_tracker_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(cone_tracker_node
  rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs vision_msgs visualization_msgs tf2_ros)
install(TARGETS cone_tracker_node DESTINATION lib/${PROJECT_NAME})

# 1. Modular Python YOLO Detector Node
install(PROGRAMS
  scripts/yolo_detector_node.py
  tools/calib_target.py
  tools/calib_capture.py
  tools/calib_core.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Package assets: config, launch, models
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
