cmake_minimum_required(VERSION 3.8)
project(Dr_QP)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
    message(WARNING "The file conanbuildinfo.cmake doesn't exist.\nRunning 'conan install' first...")
    execute_process(COMMAND conan install "${CMAKE_CURRENT_SOURCE_DIR}"
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

file(GLOB_RECURSE ALL_CODE_HDRS src/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS src/*.cpp)

add_subdirectory(tests)

add_executable(DrQP ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})

