cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME TestDrQP)
project(${PROJECT_NAME})


file(GLOB_RECURSE ALL_CODE_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(LIBS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/../../XYZRobot_Arduino/libraries)
include_directories(${LIBS_FOLDER}/HAL)
#add_subdirectory(${LIBS_FOLDER}/HAL HAL_tests)
#add_subdirectory(${LIBS_FOLDER}/BOLIDE_Player BOLIDE_Player_tests)

add_executable(${PROJECT_NAME} ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
