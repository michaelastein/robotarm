cmake_minimum_required(VERSION 3.8)
project(point_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- FIND DEPENDENCIES ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------------- EXECUTABLE 1 ----------------
add_executable(point_motion src/point_motion.cpp)

ament_target_dependencies(point_motion
  rclcpp
  moveit_ros_planning_interface
)

# ---------------- EXECUTABLE 2 ----------------
add_executable(point_motion_path src/point_motion_path.cpp)

ament_target_dependencies(point_motion_path
  rclcpp
  moveit_ros_planning_interface
)

# ---------------- EXECUTABLE 3 (VISUAL SERVO) ----------------
add_executable(point_following_2d src/point_following_2d.cpp)

ament_target_dependencies(point_following_2d
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# ---------------- INSTALL ----------------
install(TARGETS
  point_motion
  point_motion_path
  point_following_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------- TESTING ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
