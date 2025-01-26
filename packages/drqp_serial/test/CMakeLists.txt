
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

set(TARGET drqp_serial_tests)
set(TEST_NAME drqp_serial_tests)
set(EXECUTABLE "$<TARGET_FILE:${TARGET}>")
set(RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.catch2.xml")
set(OUTPUT_FILE "${CMAKE_BINARY_DIR}/catch_ros2/${TEST_NAME}.txt")
set(CMD ${EXECUTABLE}
  --reporter JUnit::out=${RESULT_FILE}
  --reporter console::out=-::colour-mode=ansi
  --out ${OUTPUT_FILE}
)

ament_add_test(
  ${TEST_NAME}
  COMMAND ${CMD}
  RESULT_FILE ${RESULT_FILE}
  OUTPUT_FILE ${OUTPUT_FILE}
)

# Copy test data to build directory as it is the default working directory at runtime
file(
  INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/test_data
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
