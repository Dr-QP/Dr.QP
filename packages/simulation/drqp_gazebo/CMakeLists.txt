cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
