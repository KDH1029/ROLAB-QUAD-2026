cmake_minimum_required(VERSION 3.8)
project(mission_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(mission_fsm_node src/mission_fsm_node.cpp)
ament_target_dependencies(mission_fsm_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  unitree_api
)

# Route sequencer: saved map points -> EGO-Planner goals, stop line / parking actions
add_executable(route_runner_node src/route_runner_node.cpp)
target_include_directories(route_runner_node PRIVATE include)
ament_target_dependencies(route_runner_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  unitree_api
)
# ROS-free test of the sequencing logic with a simulated robot
add_executable(test_route_logic src/test_route_logic.cpp)
target_include_directories(test_route_logic PRIVATE include)

install(TARGETS
  mission_fsm_node
  route_runner_node
  test_route_logic
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
