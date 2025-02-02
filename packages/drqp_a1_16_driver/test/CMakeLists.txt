
find_package(catch_ros2 REQUIRED)
add_executable(drqp_a1_16_driver_tests
  TestXYZrobotServo.cpp
)

# https://docs.ros.org/en/humble/p/catch_ros2/index.html
target_link_libraries(drqp_a1_16_driver_tests PRIVATE
  drqp_a1_16_driver
  catch_ros2::catch_ros2_with_main
)
set(TEST_DATA_DIR_IN_SOURCE_TREE ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
target_compile_definitions(drqp_a1_16_driver_tests PRIVATE TEST_DATA_DIR_IN_SOURCE_TREE="${TEST_DATA_DIR_IN_SOURCE_TREE}")

add_catch2_unit_test(drqp_a1_16_driver_tests)

# Copy test data to build directory as it is the default working directory at runtime
file(
  INSTALL ${TEST_DATA_DIR_IN_SOURCE_TREE}
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
