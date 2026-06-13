cmake_minimum_required(VERSION 3.8)
project(robotarm_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(robotarm_hardware SHARED
  src/robotarm_system.cpp
)
target_link_libraries(robotarm_hardware
  gpiod
)
target_include_directories(robotarm_hardware PUBLIC
  include
)

ament_target_dependencies(robotarm_hardware
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  hardware_interface
  robotarm_hardware.xml
)

install(
  TARGETS robotarm_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robotarm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
