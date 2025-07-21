cmake_minimum_required(VERSION 3.8)
project(jaka_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(jaka_planner REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED CONFIG)
find_package(ament_index_cpp REQUIRED)

# 添加include目录
include_directories(include)

# 添加可执行文件
file(GLOB MANIPULATION_NODE_SRC
  "src/manipulation/*.cpp"
)
add_executable(manipulation_node
  ${MANIPULATION_NODE_SRC}
)
target_link_libraries(manipulation_node
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(manipulation_node
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  shape_msgs
  jaka_msgs
  jaka_planner
  std_srvs
  ament_index_cpp
)

file(GLOB ALSON_CLIENT_SRC
  "src/alson/*.cpp"
)

add_executable(alson_client_node
  ${ALSON_CLIENT_SRC}
  src/alson/main.cpp
)
target_link_libraries(alson_client_node nlohmann_json::nlohmann_json)

ament_target_dependencies(alson_client_node
  rclcpp
  std_msgs
)

# 安装可执行文件
install(TARGETS
  manipulation_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装 config 文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 scene 目录（确保场景文件目录存在）
install(DIRECTORY config/scene/
  DESTINATION share/${PROJECT_NAME}/config/scene
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
