cmake_minimum_required(VERSION 3.8)
project(robin_kinematic_model_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robin_core_cpp REQUIRED)

include_directories(include)
add_library(${PROJECT_NAME} src/kinematic_model.cpp)
target_link_libraries(${PROJECT_NAME} robin_core_cpp::robin_core_cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)  
  ament_add_gtest(test_kinematic_model test/test_kinematic_model.cpp)
  target_link_libraries(test_kinematic_model ${PROJECT_NAME})
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(robin_core_cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
