cmake_minimum_required(VERSION 3.12)
project(adi_tmcm_bb4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core SerialBus)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core SerialBus)

set(DEPS
  rclcpp
  rclcpp_components
  rcpputils
  rcutils
  rmw
  rmw_implementation
  std_msgs
  action_msgs
  moveit_msgs
  sensor_msgs
)

set(PROJECT_SOURCES
  src/tmcm_bb4_canopen.cpp
  src/TmcmBb4CanOpen.cpp
  include/${PROJECT_NAME}/TmcmBb4CanOpen.hpp
  src/CanOpen.cpp
  include/${PROJECT_NAME}/CanOpen.hpp
  src/Tmcm1637CanOpen.cpp
  include/${PROJECT_NAME}/Tmcm1637CanOpen.hpp
)

set(EXE_NAME tmcm_bb4_canopen)

add_executable(${EXE_NAME}
  ${PROJECT_SOURCES}
)

target_include_directories(${EXE_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt${QT_VERSION_MAJOR}Core_INCLUDE_DIRS}
  ${Qt${QT_VERSION_MAJOR}SerialBus_INCLUDE_DIRS}
)

ament_target_dependencies(${EXE_NAME} ${DEPS})

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

target_link_libraries(${EXE_NAME}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::SerialBus
)

install(TARGETS
  ${EXE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
