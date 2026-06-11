find_package(catch_ros2 REQUIRED)

add_executable(drqp_joy_tests
  TestGameControllerUtils.cpp
)
drqp_test_enable_coverage(drqp_joy_tests)

target_include_directories(drqp_joy_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(drqp_joy_tests PRIVATE
  catch_ros2::catch_ros2_with_main
)

add_catch2_unit_test(drqp_joy_tests)
