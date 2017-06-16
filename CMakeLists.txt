cmake_minimum_required(VERSION 3.8)
project(Dr_QP)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


file(GLOB_RECURSE ALL_CODE_HDRS src/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS src/*.cpp)

set(LIBS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/../XYZRobot_Arduino/libraries)
include_directories(${LIBS_FOLDER}/HAL)
add_subdirectory(${LIBS_FOLDER}/HAL HAL)
add_subdirectory(${LIBS_FOLDER}/BOLIDE_Player BOLIDE_Player)

add_subdirectory(tests tests)

add_executable(DrQP ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})

