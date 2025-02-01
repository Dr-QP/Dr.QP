
find_package(catch_ros2 REQUIRED)
add_executable(drqp_serial_tests
  TestSerialRecordingProxy.cpp
)

# https://docs.ros.org/en/humble/p/catch_ros2/index.html
target_link_libraries(drqp_serial_tests PRIVATE
  drqp_serial
  catch_ros2::catch_ros2_with_main
)

add_catch2_unit_test(drqp_serial_tests)

# Copy test data to build directory as it is the default working directory at runtime
file(
  INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/test_data
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
