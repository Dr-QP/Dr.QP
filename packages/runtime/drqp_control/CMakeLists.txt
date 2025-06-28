cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package_xml()
foreach(dep ${${PROJECT_NAME}_BUILD_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(drqp_control INTERFACE)
target_compile_features(drqp_control INTERFACE cxx_std_20)

target_include_directories(drqp_control INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(control src/Control.cpp)
target_link_libraries(control drqp_control)
ament_target_dependencies(control drqp_a1_16_driver)

add_executable(pose_to_joint_state src/PoseToJointsState.cpp)
target_link_libraries(pose_to_joint_state drqp_control)
ament_target_dependencies(pose_to_joint_state ${${PROJECT_NAME}_BUILD_DEPENDS})

add_executable(pose_reader
  src/PoseReader.cpp
)
target_link_libraries(pose_reader drqp_control yaml-cpp)
ament_target_dependencies(pose_reader ${${PROJECT_NAME}_BUILD_DEPENDS})

add_executable(pose_setter
  src/PoseSetter.cpp
)
target_link_libraries(pose_setter drqp_control yaml-cpp)
ament_target_dependencies(pose_setter ${${PROJECT_NAME}_BUILD_DEPENDS})


add_subdirectory(test)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install executables for `ros2 run`
install(TARGETS
  control
  pose_to_joint_state
  pose_reader
  pose_setter
  DESTINATION lib/${PROJECT_NAME})

# Install libraries
install(TARGETS
  drqp_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # These executables will be runnable directly, aka added to PATH
)

# Install configs
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})


if(BUILD_TESTING)
  include(../../cmake/ClangFormatConfig.cmake)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
