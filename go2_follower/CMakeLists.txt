cmake_minimum_required(VERSION 3.5)
project(go2_follower)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/target_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

