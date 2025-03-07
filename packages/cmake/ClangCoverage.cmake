# Enable clang's Source based code coverage

option(DRQP_ENABLE_COVERAGE "Enable coverage" OFF)

if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR "Coverage is only supported with Clang")
endif()

function(drqp_enable_coverage TARGET)
  if (NOT DRQP_ENABLE_COVERAGE)
    return()
  endif()

  message(STATUS "Enabling coverage for ${TARGET}")
  target_compile_options(${TARGET} PRIVATE -fprofile-instr-generate -fcoverage-mapping)
endfunction()


function(drqp_test_enable_coverage TARGET)
  if (NOT DRQP_ENABLE_COVERAGE)
    return()
  endif()

  message(STATUS "Enabling coverage for ${TARGET}")
  target_link_options(${TARGET} PRIVATE -fprofile-instr-generate -fcoverage-mapping)
endfunction()
