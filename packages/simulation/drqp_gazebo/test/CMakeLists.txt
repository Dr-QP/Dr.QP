find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include(../../../cmake/RosIsolatedLaunchTest.cmake)

ament_add_pytest_test(
  test_gazebo_ci_policy
  test_gazebo_ci_policy.py
  APPEND_ENV
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)

ament_add_pytest_test(
  test_robot_control_test_support
  test_robot_control_test_support.py
  APPEND_ENV
    PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
)

set(drqp_gazebo_smoke_launch_tests
  test_robot_control_spawn.py
  test_robot_smoke.py
  test_balance_board_world.py
)

set(drqp_gazebo_full_launch_tests
  test_robot_control_stationary_posture.py
  test_balance_board_armed.py
  test_balance_board_motion_response.py
  test_locomotion_direction_reversal.py
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
if("$ENV{DRQP_TEST_MODE}" STREQUAL "slow")
  list(APPEND drqp_gazebo_launch_tests ${drqp_gazebo_full_launch_tests})
endif()

set(drqp_gazebo_launch_test_timeout 1800)
foreach(test_file IN LISTS drqp_gazebo_launch_tests)
  # The harness owns the physics deadlines. This wall-clock backstop prevents a
  # wedged executable from consuming the complete slow-job budget.
  drqp_add_ros_isolated_launch_test(
    ${test_file}
    TIMEOUT ${drqp_gazebo_launch_test_timeout}
    APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endforeach()
