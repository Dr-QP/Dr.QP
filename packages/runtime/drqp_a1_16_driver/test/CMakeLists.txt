
find_package(catch_ros2 REQUIRED)
add_executable(drqp_a1_16_driver_tests
  TestXYZrobotServo.cpp
)
drqp_test_enable_coverage(drqp_a1_16_driver_tests)

# https://docs.ros.org/en/jazzy/p/catch_ros2/index.html
target_link_libraries(drqp_a1_16_driver_tests PRIVATE
  drqp_a1_16_driver
  catch_ros2::catch_ros2
)
set(TEST_DATA_DIR_IN_SOURCE_TREE ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
target_compile_definitions(drqp_a1_16_driver_tests PRIVATE TEST_DATA_DIR_IN_SOURCE_TREE="${TEST_DATA_DIR_IN_SOURCE_TREE}")

add_catch2_unit_test(drqp_a1_16_driver_tests)

