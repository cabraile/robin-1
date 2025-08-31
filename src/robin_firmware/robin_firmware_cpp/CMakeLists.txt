cmake_minimum_required(VERSION 3.8)
project(robin_firmware_cpp)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
include_directories(include)
add_library(${PROJECT_NAME} INTERFACE)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)  
endif()

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

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
