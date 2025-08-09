
find_package(ament_cmake_ros REQUIRED)
find_package(launch_testing_ament_cmake REQUIRED)
function(add_ros_isolated_launch_test path)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
endfunction()


find_package(catch_ros2 REQUIRED)
add_executable(drqp_control_tests
  TestRobotConfig.cpp
)
drqp_test_enable_coverage(drqp_control_tests)

# https://docs.ros.org/en/jazzy/p/catch_ros2/index.html
target_link_libraries(drqp_control_tests PRIVATE
  drqp_control
  catch_ros2::catch_ros2_with_main
)
set(TEST_DATA_DIR_IN_SOURCE_TREE ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
target_compile_definitions(drqp_control_tests PRIVATE TEST_DATA_DIR_IN_SOURCE_TREE="${TEST_DATA_DIR_IN_SOURCE_TREE}")

add_catch2_unit_test(drqp_control_tests)
