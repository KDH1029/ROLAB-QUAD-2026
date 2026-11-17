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
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Cone localisation & tracking from the LiDAR alone (cone_lidar.hpp) -> world frame
add_executable(cone_tracker_node src/cone_tracker_node.cpp)
target_include_directories(cone_tracker_node PRIVATE ${EIGEN3_INCLUDE_DIRS} include)
add_executable(test_cone_lidar src/test_cone_lidar.cpp)
target_include_directories(test_cone_lidar PRIVATE ${EIGEN3_INCLUDE_DIRS} include)
ament_target_dependencies(cone_tracker_node
  rclcpp rclcpp_lifecycle sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros)
install(TARGETS cone_tracker_node test_cone_lidar DESTINATION lib/${PROJECT_NAME})

# Package assets: config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
