cmake_minimum_required(VERSION 3.16)
project(auto_charge VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
    src/main.cpp
    src/mainwindow.cpp
    include/mainwindow.h
    src/mainwindow.ui
)

qt_add_executable(auto_charge
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)

target_include_directories(auto_charge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(auto_charge PRIVATE Qt6::Widgets)
ament_target_dependencies(auto_charge PUBLIC 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS auto_charge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(auto_charge)
endif()

ament_package()
