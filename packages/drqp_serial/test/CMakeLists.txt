
find_package(catch_ros2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread serialization system)

add_executable(drqp_serial_tests
  TestSerialRecordingProxy.cpp
)

# https://docs.ros.org/en/humble/p/catch_ros2/index.html
target_link_libraries(drqp_serial_tests PRIVATE
  drqp_serial
  catch_ros2::catch_ros2_with_main
  Boost::thread
  Boost::serialization
  Boost::system
)

ament_add_test(
  drqp_serial_tests
  COMMAND $<TARGET_FILE:drqp_serial_tests> --reporter JUnit::out=result-junit.xml --reporter console::out=-::colour-mode=ansi
)
