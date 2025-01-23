cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_a1_16_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(drqp_serial REQUIRED)
find_package(rclcpp REQUIRED)

add_library(drqp_a1_16_driver STATIC
    include/drqp_a1_16_driver/XYZrobotServo.h

    src/XYZrobotServo.cpp
)

target_compile_features(drqp_a1_16_driver PUBLIC cxx_std_17)
target_include_directories(drqp_a1_16_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(drqp_a1_16_driver PUBLIC rclcpp drqp_serial)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install libraries
install(TARGETS
  drqp_a1_16_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


if(BUILD_TESTING)
  set(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

  get_filename_component(CONFIG_FILE_RES "${CONFIG_FILE}" REALPATH)
  if(EXISTS ${CONFIG_FILE_RES})
    set(ament_cmake_clang_format_CONFIG_FILE "${CONFIG_FILE_RES}" CACHE STRING "Path to the clang-format file to use")
  else()
    message(FATAL_ERROR ".clang-format config \"${CONFIG_FILE_RES}\" does not exist")
  endif()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(examples)

ament_package()
