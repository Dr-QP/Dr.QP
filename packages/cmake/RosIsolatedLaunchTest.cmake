function(drqp_add_ros_isolated_launch_test path)
  set(runner "${ament_cmake_ros_DIR}/run_test_isolated.py")
  cmake_path(GET path STEM test_name)
  ament_add_pytest_test(
    "${test_name}"
    "${path}"
    RUNNER "${runner}"
    ${ARGN}
  )
endfunction()
