cmake_minimum_required(VERSION 3.16)
project(Dr_QP CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONAN_FOLLOW_BUILD_TYPE "Pass CMAKE_BUILD_TYPE to conan install" OFF)

######################################################################################################################################################
# Conan
if(NOT EXISTS ${CMAKE_BINARY_DIR}/conaninfo.txt)
  message(STATUS "The file conanbuildinfo.cmake doesn't exist.\nRunning 'conan install' first...")
  set(BUILD_ARGS --build=missing -s cppstd=${CMAKE_CXX_STANDARD} -s compiler.libcxx=libstdc++11)
  if (CONAN_FOLLOW_BUILD_TYPE)
    set(BUILD_ARGS ${BUILD_ARGS} -s build_type=${CMAKE_BUILD_TYPE})
  endif()
  execute_process(COMMAND conan install "${CMAKE_CURRENT_SOURCE_DIR}" ${BUILD_ARGS}
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
######################################################################################################################################################

if(UNIX AND NOT APPLE)
  set(LINUX TRUE)
endif()

add_compile_options(-Werror)
add_compile_options(-fvisibility=hidden -fvisibility-inlines-hidden)

add_subdirectory(src)
add_subdirectory(tests)

