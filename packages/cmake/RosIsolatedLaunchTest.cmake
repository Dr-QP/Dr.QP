function(drqp_add_ros_isolated_launch_test path)
  set(runner "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test(
    "${path}"
    RUNNER "${runner}"
    ${ARGN}
  )
endfunction()
