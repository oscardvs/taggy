cmake_minimum_required(VERSION 3.5)
project(go2_perception)

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

# ============================================================
# FIND DEPENDENCIES
# ============================================================

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# TensorRT - search common installation paths
set(TENSORRT_INCLUDE_DIRS "")
set(TENSORRT_LIBRARIES "")

# Try to find TensorRT headers
foreach(tensorrt_path 
    "/usr/include/x86_64-linux-gnu"
    "/usr/include/aarch64-linux-gnu"
    "/usr/local/cuda/include"
    "/usr/include"
    "$ENV{HOME}/TensorRT/include")
  if(EXISTS "${tensorrt_path}/NvInfer.h")
    set(TENSORRT_INCLUDE_DIRS ${tensorrt_path})
    break()
  endif()
endforeach()

# Try to find TensorRT libraries
foreach(tensorrt_lib_path
    "/usr/lib/x86_64-linux-gnu"
    "/usr/lib/aarch64-linux-gnu"
    "/usr/local/cuda/lib64"
    "/usr/lib"
    "$ENV{HOME}/TensorRT/lib")
  if(EXISTS "${tensorrt_lib_path}/libnvinfer.so")
    set(TENSORRT_LIB_DIR ${tensorrt_lib_path})
    break()
  endif()
endforeach()

if(TENSORRT_INCLUDE_DIRS AND TENSORRT_LIB_DIR)
  message(STATUS "Found TensorRT:")
  message(STATUS "  Include: ${TENSORRT_INCLUDE_DIRS}")
  message(STATUS "  Lib dir: ${TENSORRT_LIB_DIR}")
  
  set(TENSORRT_LIBRARIES
    ${TENSORRT_LIB_DIR}/libnvinfer.so
    ${TENSORRT_LIB_DIR}/libnvonnxparser.so
    ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so
  )
else()
  message(FATAL_ERROR "TensorRT not found! Please install TensorRT or set TENSORRT_ROOT")
endif()

# ============================================================
# INCLUDE DIRECTORIES
# ============================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
)

# ============================================================
# PERCEPTION NODE EXECUTABLE
# ============================================================
add_executable(perception_node
  src/perception_node.cpp
)

target_link_libraries(perception_node
  ${OpenCV_LIBRARIES}
  ${TENSORRT_LIBRARIES}
  ${CUDA_LIBRARIES}
)

ament_target_dependencies(perception_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ============================================================
# INSTALL TARGETS
# ============================================================

# Install executable
install(TARGETS
  perception_node
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

# ============================================================
# TESTING
# ============================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

