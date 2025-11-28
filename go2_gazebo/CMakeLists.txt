cmake_minimum_required(VERSION 3.5)
project(go2_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  config
  rviz
  xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

