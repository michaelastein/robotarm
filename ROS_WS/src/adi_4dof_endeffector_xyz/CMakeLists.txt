cmake_minimum_required(VERSION 3.12)
project(adi_4dof_endeffector_xyz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(endeffector_xyz src/endeffector_xyz.cpp)
ament_target_dependencies(endeffector_xyz rclcpp visualization_msgs)

add_executable(endeffector_listener src/endeffector_listener.cpp)
ament_target_dependencies(endeffector_listener rclcpp visualization_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  endeffector_xyz
  endeffector_listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
