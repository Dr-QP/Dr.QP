cmake_minimum_required(VERSION 3.8)
project(drqp_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(
    test_moveit_config_package
    test/test_moveit_config_package.py
  )

  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test(
    test/test_moveit_runtime_launch.py
    RUNNER "${RUNNER}"
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
