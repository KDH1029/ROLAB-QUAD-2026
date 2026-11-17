cmake_minimum_required(VERSION 3.8)
project(go2_wall_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Build wall_runner_node executable
add_executable(wall_runner_node src/wall_runner_node.cpp)
ament_target_dependencies(wall_runner_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Install executable
install(TARGETS
  wall_runner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install python helper scripts
install(PROGRAMS
  scripts/patrol_monitor.py
  scripts/go2_sport_bridge.py
  scripts/patrol_udp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config, launch, and rviz directories
install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
