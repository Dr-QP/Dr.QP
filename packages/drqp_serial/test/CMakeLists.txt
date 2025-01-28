
find_package(catch_ros2 REQUIRED)
add_executable(drqp_serial_tests
  TestSerialRecordingProxy.cpp
)

# https://docs.ros.org/en/humble/p/catch_ros2/index.html
target_link_libraries(drqp_serial_tests PRIVATE
  drqp_serial
  catch_ros2::catch_ros2_with_main
)

function(add_catch2_unit_test TARGET)
  cmake_parse_arguments(ARG
    ""
    "EXECUTABLE;RESULT_FILE;OUTPUT_FILE"
    "EXTRA_COMMAND"
    ${ARGN})

  if(ARG_EXECUTABLE)
    set(EXECUTABLE "${ARG_EXECUTABLE}")
  else()
    set(EXECUTABLE "$<TARGET_FILE:${TARGET}>")
  endif()

  if(ARG_RESULT_FILE)
    set(RESULT_FILE "${ARG_RESULT_FILE}")
  else()
    set(RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}.catch2.xml")
  endif()

  if(ARG_OUTPUT_FILE)
    set(OUTPUT_FILE "${ARG_OUTPUT_FILE}")
  else()
    set(OUTPUT_FILE "${CMAKE_BINARY_DIR}/catch2_test_results/${TARGET}.txt.ans")
  endif()

  set(CMD ${EXECUTABLE}
    --reporter JUnit::out=${RESULT_FILE}
    --reporter console::out=-::colour-mode=ansi
    ${ARG_EXTRA_COMMAND}
  )

  ament_add_test(
    ${TARGET}
    COMMAND ${CMD}
    RESULT_FILE ${RESULT_FILE}
    OUTPUT_FILE ${OUTPUT_FILE}
  )
endfunction()

add_catch2_unit_test(drqp_serial_tests)

# Copy test data to build directory as it is the default working directory at runtime
file(
  INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/test_data
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)
