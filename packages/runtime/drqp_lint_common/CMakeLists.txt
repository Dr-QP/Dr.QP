cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_lint_common NONE)

find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_export_dependencies REQUIRED)

ament_package_xml()
ament_export_dependencies(${${PROJECT_NAME}_EXEC_DEPENDS})
ament_package()
