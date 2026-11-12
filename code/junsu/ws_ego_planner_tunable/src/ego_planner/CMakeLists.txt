cmake_minimum_required(VERSION 3.8)
project(ego_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ego_planner_core
  src/grid_map.cpp
  src/a_star.cpp
  src/bspline_optimizer.cpp
)
ament_target_dependencies(ego_planner_core
  sensor_msgs
)
target_include_directories(ego_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(ego_planner_node src/ego_planner_node.cpp)
target_link_libraries(ego_planner_node ego_planner_core)
ament_target_dependencies(ego_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(test_abpolicy src/test_abpolicy.cpp)
target_link_libraries(test_abpolicy ego_planner_core)
ament_target_dependencies(test_abpolicy
  sensor_msgs
)

install(TARGETS
  ego_planner_core
  ego_planner_node
  test_abpolicy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

