cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# 设置库目录
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加库目录到搜索路径
link_directories(${LIBRARY_DIR})

# 创建可执行文件
add_executable(robot_controller_node 
  src/robot_controller_node.cpp
  src/robot_controller.cpp
)

target_include_directories(robot_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_controller_node PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++17

ament_target_dependencies(
  robot_controller_node
  "rclcpp"
  "rclcpp_action"
  "control_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "jaka_msgs"
  "std_srvs"
)

# 链接 JAKA SDK 库
target_link_libraries(robot_controller_node ${LIBRARY_DIR}/libjakaAPI.so)

install(TARGETS robot_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

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

ament_package()
