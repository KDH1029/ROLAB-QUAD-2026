cmake_minimum_required(VERSION 3.8)
project(wall_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(wall_filter_node src/wall_filter_node.cpp)
ament_target_dependencies(wall_filter_node
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS
  wall_filter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
