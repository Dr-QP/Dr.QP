cmake_minimum_required(VERSION 3.11)
project(Dr_QP CXX C)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  message(
    WARNING
      "The file conanbuildinfo.cmake doesn't exist.\nRunning 'conan install' first..."
    )
  execute_process(COMMAND conan install "${CMAKE_CURRENT_SOURCE_DIR}"
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Werror)

add_subdirectory(src)
add_subdirectory(tests)

