cmake_minimum_required(VERSION 3.5)
project(go2_mission)

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
include_directories(include)

# Dependencies
set(DEPENDENCY_LIST
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  unitree_go
  nav2_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Mission Executive Node
add_executable(mission_executive_node
  src/mission_executive_node.cpp
)
ament_target_dependencies(mission_executive_node ${DEPENDENCY_LIST})

# Visual Servo Node (IBVS controller with LiDAR safety)
add_executable(visual_servo_node
  src/visual_servo_node.cpp
)
ament_target_dependencies(visual_servo_node ${DEPENDENCY_LIST})

# Install targets
install(TARGETS 
  mission_executive_node
  visual_servo_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  scripts/voice_command_node.py
  scripts/test_voice_command.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

