cmake_minimum_required(VERSION 3.8)

include("cmake/HunterGate.cmake")

# 1. Try to detect hunter by:
#   * cmake variable
#   * environment variable
#   * in HOME (`.hunter` directory)
#   * in SYSTEMDRIVE (`.hunter` directory, windows only)
#   * in USERPROFILE (`.hunter` directory, windows only)
# 2. if failed, download it
# 3. set HUNTER_GATE_* variables
# 4. include master file: 'include(${HUNTER_SELF}/cmake/Hunter)
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.18.12.tar.gz"
    SHA1 "7e8c74787e08c476484d3e5106524fe7a5a0cc56"
)

project(Dr_QP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

hunter_add_package(Boost COMPONENTS thread serialization system unit_test_framework)

file(GLOB_RECURSE ALL_CODE_HDRS src/*.h)
file(GLOB_RECURSE ALL_CODE_SRCS src/*.cpp)

set(LIBS_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/../XYZRobot_Arduino/libraries)
include_directories(${LIBS_FOLDER}/HAL)
add_subdirectory(${LIBS_FOLDER}/HAL HAL)
add_subdirectory(${LIBS_FOLDER}/BOLIDE_Player BOLIDE_Player)

add_subdirectory(tests)
add_subdirectory(UnixHAL)

add_executable(DrQP ${ALL_CODE_HDRS} ${ALL_CODE_SRCS} ${SOURCE_FILES})

