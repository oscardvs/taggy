cmake_minimum_required(VERSION 3.5)
project(go2_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/go2_velocity_controller.py
  scripts/go2_odometry.py
  scripts/go2_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Rename scripts to remove .py extension for ROS2 convention
install(PROGRAMS
  scripts/go2_velocity_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_velocity_controller
)

install(PROGRAMS
  scripts/go2_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_odometry
)

install(PROGRAMS
  scripts/go2_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go2_state_publisher
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

