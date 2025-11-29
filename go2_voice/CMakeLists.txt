cmake_minimum_required(VERSION 3.5)
project(go2_voice)

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

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install documentation
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

# Install Python scripts
install(PROGRAMS
  scripts/voice_command_node.py
  scripts/test_voice_command.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

