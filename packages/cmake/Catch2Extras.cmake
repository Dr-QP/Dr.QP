
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
    set(RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TARGET}-catch2.xunit.xml")
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
