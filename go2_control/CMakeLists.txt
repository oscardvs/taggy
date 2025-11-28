cmake_minimum_required(VERSION 3.5)
project(go2_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(include include/common include/nlohmann)

# Dependencies
set(DEPENDENCY_LIST
  rclcpp
  std_msgs
  geometry_msgs
  unitree_go
  unitree_api
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

# Main control node executable
add_executable(go2_control_node
  src/go2_control_node.cpp
  src/common/sport_client.cpp
)
ament_target_dependencies(go2_control_node ${DEPENDENCY_LIST})

# Install targets
install(TARGETS 
  go2_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

