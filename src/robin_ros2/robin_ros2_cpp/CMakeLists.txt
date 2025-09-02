cmake_minimum_required(VERSION 3.8)
project(robin_ros2_cpp)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robin_firmware_cpp REQUIRED)
find_package(robin_kinematic_model_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(robin_platform_interface_node src/robin_platform_interface_node.cpp)
ament_target_dependencies(robin_platform_interface_node 
  cv_bridge
  geometry_msgs
  OpenCV 
  rclcpp 
  robin_firmware_cpp 
  robin_kinematic_model_cpp
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)  
endif()

install(
  TARGETS
    robin_platform_interface_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
