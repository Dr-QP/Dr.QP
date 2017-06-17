cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME UnixHAL)
project(${PROJECT_NAME})

file(GLOB_RECURSE ALL_CODE_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library(${PROJECT_NAME} ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

find_package(Boost 1.63 REQUIRED thread system)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
