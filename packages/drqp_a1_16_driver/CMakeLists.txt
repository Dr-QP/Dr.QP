cmake_minimum_required(VERSION 3.8)
project(drqp_a1_16_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(drqp_serial REQUIRED)

add_library(drqp_a1_16_driver STATIC
    include/drqp_a1_16_driver/XYZrobotServo.h

    src/XYZrobotServo.cpp
)

target_compile_features(drqp_a1_16_driver PUBLIC cxx_std_17)
target_include_directories(drqp_a1_16_driver PUBLIC include)
ament_target_dependencies(drqp_a1_16_driver PUBLIC drqp_serial)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(examples)

ament_package()
