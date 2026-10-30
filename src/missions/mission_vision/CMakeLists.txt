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

# 1. Modular Python YOLO Detector Node
install(PROGRAMS
  scripts/yolo_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 2. Package assets: config, launch, models
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
