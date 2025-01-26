
find_package(catch_ros2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread serialization system)

add_executable(drqp_serial_test_catch
  drqp_serial_test.cpp
)

# https://docs.ros.org/en/humble/p/catch_ros2/index.html
target_link_libraries(drqp_serial_test_catch PRIVATE
  catch_ros2::catch_ros2_with_main
  drqp_serial
)

add_executable(drqp_serial_tests
  TestSerialRecordingProxy.cpp
  TestsMain.cpp
)

target_link_libraries(drqp_serial_tests PRIVATE drqp_serial Boost::thread Boost::serialization Boost::system)
