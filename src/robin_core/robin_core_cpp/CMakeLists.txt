cmake_minimum_required(VERSION 3.8)
project(robin_core_cpp)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only library
include_directories(include)
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(Eigen3)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install INTERFACE target
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED) 

  ament_add_gtest(test_core
    test/test_core.cpp
  )
  target_link_libraries(test_core ${PROJECT_NAME}) 
endif()

ament_package()