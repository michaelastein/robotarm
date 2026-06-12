cmake_minimum_required(VERSION 3.8)
project(robotarm_fake_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(robotarm_fake_hardware SHARED
  src/velocity_fake_system.cpp
)

target_include_directories(robotarm_fake_hardware PUBLIC
  include
)

ament_target_dependencies(robotarm_fake_hardware
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  hardware_interface
  robotarm_fake_hardware.xml
)

install(
  TARGETS robotarm_fake_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robotarm_fake_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
