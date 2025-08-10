cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include(../../cmake/ClangCoverage.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package_xml()
foreach(dep ${${PROJECT_NAME}_BUILD_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(drqp_control STATIC
  include/drqp_control/RobotConfig.h
  src/RobotConfig.cpp
)
target_compile_features(drqp_control PUBLIC cxx_std_20)
target_link_libraries(drqp_control yaml-cpp)
ament_target_dependencies(drqp_control ${${PROJECT_NAME}_BUILD_DEPENDS})
drqp_library_enable_coverage(drqp_control)

target_include_directories(drqp_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(control src/Control.cpp)
target_link_libraries(control drqp_control)
ament_target_dependencies(control drqp_a1_16_driver)

add_library(drqp_ros2_control
  SHARED

  include/drqp_control/a1_16_hardware_interface.h
  src/a1_16_hardware_interface.cpp
)

target_link_libraries(drqp_ros2_control drqp_control)
ament_target_dependencies(drqp_ros2_control ${${PROJECT_NAME}_BUILD_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface drqp_control_plugin.xml)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install executables for `ros2 run`
install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(TARGETS
  drqp_control
  drqp_ros2_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # These executables will be runnable directly, aka added to PATH
)

# Install configs
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})


if(BUILD_TESTING)
  include(../../cmake/ClangFormatConfig.cmake)
  include(../../cmake/Catch2Extras.cmake)

  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  #TODO: Add gmock tests for hardware interface from
  # https://github.com/ros-controls/ros2_control/blob/jazzy/hardware_interface/CMakeLists.txt

endif()

ament_package()
