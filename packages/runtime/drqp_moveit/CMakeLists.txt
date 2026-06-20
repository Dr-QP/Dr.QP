cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_moveit)

include(../../cmake/RosIsolatedLaunchTest.cmake)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # MoveIt launch tests spin up a full simulator/move_group stack and can
  # legitimately take several minutes, so allow at least 5 minutes before
  # timing out.
  set(drqp_moveit_test_timeout 300)

  set(drqp_moveit_launch_tests
    test/test_move_group_launch_smoke.py
    test/test_demo_launch_smoke.py
    test/test_demo_gazebo_launch_smoke.py
  )

  foreach(test_file IN LISTS drqp_moveit_launch_tests)
    drqp_add_ros_isolated_launch_test(
      ${test_file}
      TIMEOUT ${drqp_moveit_test_timeout}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
    )
  endforeach()

  # Full runtime launch exercises a real motion and needs a longer budget.
  drqp_add_ros_isolated_launch_test(
    test/test_moveit_runtime_launch.py
    TIMEOUT 600
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
