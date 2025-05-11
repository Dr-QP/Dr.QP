
find_package(catch_ros2 REQUIRED)

add_executable(drqp_serial_tests
  TestSerialPort.cpp
  TestSerialRecordingProxy.cpp
)
drqp_test_enable_coverage(drqp_serial_tests)

# https://docs.ros.org/en/jazzy/p/catch_ros2/index.html
target_link_libraries(drqp_serial_tests PRIVATE
  drqp_serial
  catch_ros2::catch_ros2_with_main
)

target_compile_definitions(drqp_serial_tests PRIVATE
  TEST_DATA_DIR_IN_SOURCE_TREE="${CMAKE_CURRENT_SOURCE_DIR}/test_data"
  TEST_DATA_DIR_IN_BUILD_TREE="${CMAKE_CURRENT_BINARY_DIR}/"
)

add_catch2_unit_test(drqp_serial_tests)
