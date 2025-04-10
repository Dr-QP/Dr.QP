cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_interfaces)

cmake_policy(SET CMP0148 OLD) # Re-enable FindPythonInterp needed by rosidl_generate_interfaces

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package_xml()
foreach(dep ${${PROJECT_NAME}_BUILD_DEPENDS} ${${PROJECT_NAME}_BUILDTOOL_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/ServoPositionGoal.msg
  msg/MultiServoPositionGoal.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})

ament_package()
