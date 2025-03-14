cmake_minimum_required(VERSION 3.16.3)
project(adi_4dof_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)

set(DEPS
  rclcpp
  moveit_core
  pluginlib
  class_loader
  urdf
  tf2_kdl
  )

set(ADI_4DOF_IK_PLUGIN_LIB adi_4dof_ik_kinematics_plugin)

add_library(${ADI_4DOF_IK_PLUGIN_LIB} SHARED src/adi_4dof_ik_kinematics_plugin.cpp src/SerialRobot.cpp)

target_link_libraries(${ADI_4DOF_IK_PLUGIN_LIB})

target_include_directories(${ADI_4DOF_IK_PLUGIN_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies(${ADI_4DOF_IK_PLUGIN_LIB} ${DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${ADI_4DOF_IK_PLUGIN_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

target_compile_definitions(${ADI_4DOF_IK_PLUGIN_LIB} PRIVATE "ADI_4DOF_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

pluginlib_export_plugin_description_file(moveit_core adi_4dof_ik_kinematics_description.xml)

ament_export_libraries(${ADI_4DOF_IK_PLUGIN_LIB})
ament_export_targets( export_${PROJECT_NAME} )
ament_export_dependencies( ${DEPS} )
ament_export_include_directories(include)

ament_package()

