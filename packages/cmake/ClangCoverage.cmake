# Enable clang's Source based code coverage

option(DRQP_ENABLE_COVERAGE "Enable coverage" OFF)

if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR "Coverage is only supported with Clang")
endif()

set(CLANG_FLAGS -fprofile-instr-generate -fcoverage-mapping)

# Enable coverage for C++ library
# by adding needed compilation flags
function(drqp_library_enable_coverage TARGET)
  if (NOT DRQP_ENABLE_COVERAGE)
    return()
  endif()

  message(STATUS "DRQP: Enabling coverage for source code of ${TARGET} library")
  target_compile_options(${TARGET} PRIVATE ${CLANG_FLAGS})
endfunction()

# Enable coverage for C++ test binary
# by adding needed linker flags flags
# code from the test binary will be ignored
function(drqp_test_enable_coverage TARGET)
  if (NOT DRQP_ENABLE_COVERAGE)
    return()
  endif()

  message(STATUS "DRQP: Enabling coverage for ${TARGET} test target, code from the test binary will be ignored")
  target_link_options(${TARGET} PRIVATE ${CLANG_FLAGS})
endfunction()

# Enable coverage for C++ executable
# by adding needed compilation and linker flags
# code from this executable will be included
function(drqp_executable_enable_coverage TARGET)
  if (NOT DRQP_ENABLE_COVERAGE)
    return()
  endif()

  message(STATUS "DRQP: Enabling coverage for executable target ${TARGET}, code from this executable will be included")
  target_compile_options(${TARGET} PRIVATE ${CLANG_FLAGS})
  target_link_options(${TARGET} PRIVATE ${CLANG_FLAGS})
endfunction()
