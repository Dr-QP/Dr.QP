cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME TestDrQP)
project(${PROJECT_NAME})

find_package(Boost 1.63 REQUIRED thread serialization system unit_test_framework)

file(GLOB_RECURSE ALL_CODE_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

include_directories(../UnixHAL)

add_definitions(-DBOOST_TEST_DYN_LINK)

add_executable(${PROJECT_NAME} ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} BOLIDE_Player UnixHAL)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

enable_testing()

include(BoostTestHelpers.cmake)
add_boost_tests(${PROJECT_NAME})