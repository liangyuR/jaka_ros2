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
find_package(jaka_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2)

# 启用 QML 语言服务器支持
set(QT_QML_GENERATE_QMLLS_INI ON)

set(PROJECT_SOURCES
    src/main.cpp
    src/auto_charge_node.cpp
    src/camera_controller.cpp
    src/robot_manager.cpp
    include/auto_charge/auto_charge_node.h
    include/auto_charge/camera_controller.h
    include/auto_charge/robot_manager.h
    include/auto_charge/robot_manager_utils.h
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/qml/qtquickcontrols2.conf
    ${CMAKE_CURRENT_BINARY_DIR}/qtquickcontrols2.conf
    COPYONLY
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/qtquickcontrols2.conf
    DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
)

qt_add_resources(AUTO_CHARGE_RESOURCES qml/resources.qrc)

qt_add_executable(auto_charge
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
    ${AUTO_CHARGE_RESOURCES}
)

target_include_directories(auto_charge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(auto_charge PRIVATE 
    Qt6::Quick
    Qt6::QuickControls2
    nlohmann_json::nlohmann_json
    yaml-cpp
)
ament_target_dependencies(auto_charge PUBLIC 
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  jaka_msgs
  ament_index_cpp
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
