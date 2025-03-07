cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_a1_16_driver)

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

add_library(drqp_a1_16_driver STATIC
    include/drqp_a1_16_driver/XYZrobotServo.h

    src/SerialFactory.cpp
    src/XYZrobotServo.cpp
)

target_compile_features(drqp_a1_16_driver PUBLIC cxx_std_17)
target_include_directories(drqp_a1_16_driver PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(drqp_a1_16_driver PUBLIC ${${PROJECT_NAME}_BUILD_DEPENDS})

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

add_executable(pose_reader src/PoseReader.cpp)
target_link_libraries(pose_reader drqp_a1_16_driver)

add_executable(pose_setter src/PoseSetter.cpp)
target_link_libraries(pose_setter drqp_a1_16_driver)

# Install executables for `ros2 run`
install(TARGETS
  pose_reader
  pose_setter
  DESTINATION lib/${PROJECT_NAME}
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
endif()

add_subdirectory(examples)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
