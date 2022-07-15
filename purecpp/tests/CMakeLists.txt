cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME TestDrQP)
project(${PROJECT_NAME})

file(GLOB_RECURSE ALL_CODE_HDRS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(doctest REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread serialization system)

add_executable(${PROJECT_NAME}
     ${ALL_CODE_HDRS}
     ${ALL_CODE_SRCS}
     ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} PRIVATE servo Boost::thread Boost::serialization Boost::system doctest::doctest)

file(INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/test_data
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tests
)
