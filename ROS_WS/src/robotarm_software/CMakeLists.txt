cmake_minimum_required(VERSION 3.8)

project(robotarm_software)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

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

ament_python_install_package(
  ${PROJECT_NAME}
)

install(
  TARGETS
    hotspot_path_planner
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    robotarm_software/safety_node.py
    robotarm_software/safety_supervisor.py
    robotarm_software/tool_position_velocity_control.py
    robotarm_software/hotspot_detector.py
    robotarm_software/hotspot_web_stream.py
    robotarm_software/start_position_servo.py
    robotarm_software/start_position_path.py
    robotarm_software/hotspot_servo.py
    robotarm_software/joint_states_relay.py
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
