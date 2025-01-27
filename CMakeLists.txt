cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_rapidjson)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_library(drqp_rapidjson INTERFACE)
target_include_directories(drqp_rapidjson INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
