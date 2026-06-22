cmake_minimum_required(VERSION 3.22)
project(moveit_3dof)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(
  DIRECTORY
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/safety_node.py
    scripts/safety_supervisor.py
    scripts/hotspot.py
    scripts/keyboard.py
    scripts/tip_velocity_control.py
    scripts/send_twist.py
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(
    FILES .setup_assistant
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
