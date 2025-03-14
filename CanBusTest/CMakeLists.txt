cmake_minimum_required(VERSION 3.16)

project(CanBusTest VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets SerialBus)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets SerialBus)

set(PROJECT_SOURCES
        main.cpp
        CanBusTest.cpp
        CanBusTest.h
        CanBusTest.ui
        CanOpen.cpp
        CanOpen.h
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)   
    qt_add_executable(CanBusTest
            MANUAL_FINALIZATION
            ${PROJECT_SOURCES}
    )
else()
    add_executable(CanBusTest
        ${PROJECT_SOURCES}
    )
endif()

target_link_libraries(CanBusTest PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::SerialBus)

set_target_properties(CanBusTest PROPERTIES
    ${BUNDLE_ID_OPTION}
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS CanBusTest
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(CanBusTest)
endif()
