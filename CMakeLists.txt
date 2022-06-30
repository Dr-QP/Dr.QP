cmake_minimum_required(VERSION 3.11)
project(Dr_QP CXX C)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  message(
    WARNING
      "The file conanbuildinfo.cmake doesn't exist.\nRunning 'conan install' first..."
    )
  execute_process(COMMAND conda run -n dr.qp conan install "${CMAKE_CURRENT_SOURCE_DIR}" --build=missing
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Werror)

add_subdirectory(src)
add_subdirectory(tests)

