cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME TestDrQP)
project(${PROJECT_NAME})

file(GLOB_RECURSE ALL_CODE_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(Boost REQUIRED COMPONENTS thread serialization system)

add_executable(${PROJECT_NAME}
               ${ALL_CODE_HDRS}
               ${ALL_CODE_SRCS}
               ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} BOLIDE_Player HALUnix)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

file(INSTALL
     ${CMAKE_CURRENT_SOURCE_DIR}/test_data
     DESTINATION
     ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/)
