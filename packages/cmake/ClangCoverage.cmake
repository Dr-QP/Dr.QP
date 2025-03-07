# Enable clang's Source based code coverage

option(DRQP_ENABLE_COVERAGE "Enable coverage" OFF)
if (NOT DRQP_ENABLE_COVERAGE)
  return()
endif()

if (NOT BUILD_TESTING)
  return()
endif()

if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR "Coverage is only supported with Clang")
endif()

message(STATUS "Enabling coverage")
add_compile_options(-fprofile-instr-generate -fcoverage-mapping)

