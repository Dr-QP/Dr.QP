cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_moveit)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test(
      "${path}"
      RUNNER "${RUNNER}"
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
      ${ARGN}
    )
  endfunction()

  add_ros_isolated_launch_test(test/test_move_group_launch_smoke.py)
  add_ros_isolated_launch_test(test/test_demo_launch_smoke.py)
  add_ros_isolated_launch_test(test/test_demo_gazebo_launch_smoke.py)
  add_ros_isolated_launch_test(test/test_moveit_runtime_launch.py)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
