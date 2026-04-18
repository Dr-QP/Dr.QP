cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(../../cmake/ClangCoverage.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drqp_interfaces REQUIRED)

# ── Shared component library ──────────────────────────────────────────────────
add_library(${PROJECT_NAME}_component SHARED
  src/game_controller.cpp
)

target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME}_component SDL3::SDL3)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  drqp_interfaces
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "drqp_joy::GameController"
  EXECUTABLE game_controller_node
)

# ── Install ───────────────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  include(../../cmake/ClangFormatConfig.cmake)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
