find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include(../../../cmake/RosIsolatedLaunchTest.cmake)

set(drqp_gazebo_smoke_launch_tests
  test_robot_control_spawn.py
  test_robot_smoke.py
  test_balance_board_world.py
)

set(drqp_gazebo_full_launch_tests
  test_robot_control_balance_mode.py
  test_balance_board_armed.py
  test_balance_board_motion_response.py
  test_imu_balance_motion.py
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

foreach(test_file IN LISTS drqp_gazebo_launch_tests)
  # Physics and behavior deadlines are enforced by simulation time in the test
  # harness. A CTest wall timeout would make the result depend on Gazebo's
  # real-time factor under parallel load, so disable that outer deadline.
  drqp_add_ros_isolated_launch_test(
    ${test_file}
    APPEND_ENV
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  cmake_path(GET test_file STEM test_name)
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 0)
endforeach()
