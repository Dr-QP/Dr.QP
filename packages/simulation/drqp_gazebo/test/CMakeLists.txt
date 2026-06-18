find_package(ament_cmake_pytest REQUIRED)
find_package(ament_cmake_ros REQUIRED)

include(../../../cmake/RosIsolatedLaunchTest.cmake)

set(drqp_gazebo_launch_tests
  test_robot_control_spawn.py
  test_robot_control_nodes_and_clock.py
  test_robot_control_movement_backward.py
  test_robot_control_movement_forward.py
  test_robot_control_movement_forward_sustained.py
  test_robot_control_movement_left.py
  test_robot_control_movement_right.py
  test_robot_control_movement_rotation.py
  test_robot_control_verify_armed_posture.py
  test_robot_control_verify_disarmed_posture.py
)

# Gazebo simulation tests spin up a full simulator and can legitimately take
# several minutes to complete, so allow at least 5 minutes before timing out.
set(drqp_gazebo_test_timeout 300)

foreach(test_file IN LISTS drqp_gazebo_launch_tests)
  drqp_add_ros_isolated_launch_test(
    ${test_file}
    TIMEOUT ${drqp_gazebo_test_timeout}
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endforeach()
