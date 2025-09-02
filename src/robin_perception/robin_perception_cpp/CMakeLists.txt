cmake_minimum_required(VERSION 3.8)
project(robin_perception_cpp)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(robin_core_cpp REQUIRED)
find_package(robin_firmware_cpp REQUIRED)

add_library(${PROJECT_NAME} 
    src/imu_filter.cpp
    src/slam_system.cpp
    src/perception_interface.cpp
)
ament_target_dependencies(${PROJECT_NAME} robin_core_cpp robin_firmware_cpp OpenCV)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export CMake target
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(robin_core_cpp robin_firmware_cpp OpenCV)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_imu_filter
    test/test_imu_filter.cpp
  )
  target_link_libraries(test_imu_filter ${PROJECT_NAME}) 
  ament_add_gtest(test_slam_system
    test/test_slam_system.cpp
  )
  target_link_libraries(test_slam_system ${PROJECT_NAME})
endif()

ament_package()
