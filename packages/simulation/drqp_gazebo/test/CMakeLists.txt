find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

include(../../../cmake/RosIsolatedLaunchTest.cmake)

set(drqp_gazebo_smoke_launch_tests
  test_robot_control_spawn.py
  test_robot_control_nodes_and_clock.py
  test_balance_board_world.py
)

set(drqp_gazebo_full_launch_tests
  test_robot_control_balance_mode.py
  test_balance_board_armed.py
  test_balance_board_motion_response.py
  test_robot_control_movement_backward.py
  test_robot_control_movement_forward.py
  test_robot_control_movement_forward_sustained.py
  test_robot_control_movement_left.py
  test_robot_control_movement_right.py
  test_robot_control_movement_rotation.py
  test_robot_control_verify_armed_posture.py
  test_robot_control_verify_disarmed_posture.py
)

set(drqp_gazebo_launch_tests ${drqp_gazebo_smoke_launch_tests})
if("$ENV{DRQP_GAZEBO_TEST_MODE}" STREQUAL "all")
  list(APPEND drqp_gazebo_launch_tests ${drqp_gazebo_full_launch_tests})
endif()

foreach(test_file IN LISTS drqp_gazebo_launch_tests)
  # Each launch test boots Gazebo and exercises several robot/board scenarios, so
  # it needs well over the 60s add_launch_test default before completing.
  drqp_add_ros_isolated_launch_test(
    ${test_file}
    TIMEOUT 300
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endforeach()
