find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)

include(../../../cmake/RosIsolatedLaunchTest.cmake)

# drqp_add_ros_isolated_launch_test → add_launch_test → python -m launch_testing.launch_test <file>
# That runner (launch_test.py:85) loads tests via LoadTestsFromPythonModule and drives them
#  with LaunchTestRunner — pure unittest machinery, no pytest involved.

# conftest.py is a pytest-only concept. Pytest discovers and executes conftest files during collection;
# unittest never touches them. The pytest_collection_modifyitems hook in your conftest will simply
# never be called in this code path.

# If you want the DRQP_GAZEBO_TEST_MODE filtering to work for launch tests, you'd need to implement
# it directly in the test files (e.g., check the env var in setUpClass or skip individual tests
# with unittest.skipUnless), or find another hook point that launch_testing supports.

set(drqp_gazebo_launch_tests
  test_robot_control_spawn.py
  test_robot_control_nodes_and_clock.py
  test_balance_board_world.py

  # Slow tests
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

set(DRQP_GAZEBO_TEST_MODE $ENV{DRQP_GAZEBO_TEST_MODE} CACHE STRING "Test mode for Dr.QP Gazebo tests" FORCE)

foreach(test_file IN LISTS drqp_gazebo_launch_tests)
  # Each launch test boots Gazebo and exercises several robot/board scenarios, so
  # it needs well over the 60s add_launch_test default before completing.
  drqp_add_ros_isolated_launch_test(
    ${test_file}
    TIMEOUT 300
    APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
      DRQP_GAZEBO_TEST_MODE=$ENV{DRQP_GAZEBO_TEST_MODE}
  )
endforeach()
