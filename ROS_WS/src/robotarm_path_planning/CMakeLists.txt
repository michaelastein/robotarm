cmake_minimum_required(VERSION 3.8)

project(robotarm_path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  hotspot_path_planner
  src/hotspot_path_planner.cpp
)

ament_target_dependencies(
  hotspot_path_planner
  rclcpp
  std_msgs
  geometry_msgs
  moveit_ros_planning_interface
)

target_compile_features(
  hotspot_path_planner
  PUBLIC
  cxx_std_17
)

install(
  TARGETS
    hotspot_path_planner
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
