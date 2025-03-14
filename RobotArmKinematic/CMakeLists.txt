cmake_minimum_required(VERSION 3.16)

project(RobotArmKinematic VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Xml)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Xml)

set(PROJECT_SOURCES
        main.cpp
        RobotArmKinematic.cpp
        RobotArmKinematic.h
        RobotArmKinematic.ui
        SerialRobot.cpp
        SerialRobot.h
        UrdfHandler.cpp
        UrdfHandler.h
        XrdfHandler.cpp
        XrdfHandler.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(RobotArmKinematic
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(RobotArmKinematic
        ${PROJECT_SOURCES}

    )
endif()

target_link_libraries(RobotArmKinematic PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Xml)

set_target_properties(RobotArmKinematic PROPERTIES
    ${BUNDLE_ID_OPTION}
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS RobotArmKinematic
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(RobotArmKinematic)
endif()
