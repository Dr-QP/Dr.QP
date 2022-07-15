cmake_minimum_required(VERSION 3.16)
project(Dr_QP CXX C)

######################################################################################################################################################
# Conan
if(NOT EXISTS ${CMAKE_BINARY_DIR}/conaninfo.txt)
  message(STATUS "The file conanbuildinfo.cmake doesn't exist.\nRunning 'conan install' first...")
  execute_process(COMMAND conan install "${CMAKE_CURRENT_SOURCE_DIR}" --build=missing
                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
######################################################################################################################################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(COMMAND cmake_policy)
#   cmake_policy(SET CMP0003 NEW)
# endif(COMMAND cmake_policy)

if(UNIX AND NOT APPLE)
  set(LINUX TRUE)
endif()

add_compile_options(-Werror)
add_compile_options(-fvisibility=hidden -fvisibility-inlines-hidden)

add_subdirectory(src)
add_subdirectory(tests)

