cmake_minimum_required(VERSION 3.8)
project(alson_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jaka_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ament_cmake_python REQUIRED)
find_package(nlohmann_json REQUIRED)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(include)

# Create the alson_client_node executable directly
add_executable(alson_client_node
  src/alson_client.cpp
  src/alson_client_node.cpp
  src/main.cpp
)

# Link libraries for alson_client_node
target_link_libraries(alson_client_node
  ${Boost_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# Link ament libraries for alson_client_node
ament_target_dependencies(alson_client_node
  rclcpp
  std_msgs
  jaka_msgs
  std_srvs
)

# Install targets
install(TARGETS
  alson_client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
install(PROGRAMS
  src/alson_client_test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
