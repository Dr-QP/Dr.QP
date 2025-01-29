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
target_include_directories(drqp_control INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_executable(relax src/Relax.cpp)
target_link_libraries(relax drqp_control)
ament_target_dependencies(relax drqp_a1_16_driver)


add_executable(read_all src/ReadAll.cpp)
target_link_libraries(read_all drqp_control)
ament_target_dependencies(read_all drqp_a1_16_driver)


# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install executables
install(TARGETS
  read_all
  relax
  DESTINATION lib/${PROJECT_NAME})

  # Install libraries
install(TARGETS
  drqp_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
