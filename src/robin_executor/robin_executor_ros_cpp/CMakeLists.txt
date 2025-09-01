cmake_minimum_required(VERSION 3.8)
project(robin_executor_ros_cpp)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robin_firmware_cpp REQUIRED)

include_directories(include)

add_executable(robin_executor_ros_node src/robin_executor_ros_node.cpp)
ament_target_dependencies(robin_executor_ros_node  rclcpp robin_firmware_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)  
endif()

install(
  TARGETS
    robin_executor_ros_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
