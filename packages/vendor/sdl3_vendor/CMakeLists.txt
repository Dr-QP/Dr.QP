cmake_minimum_required(VERSION 3.30..3.31)
project(sdl3_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build SDL3 from source, even if a system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(SDL3 QUIET CONFIG)
endif()

if(NOT SDL3_FOUND)
  include(ExternalProject)

  externalproject_add(SDL3-3.4.4
    PREFIX SDL3-3.4.4
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.4.4
    UPDATE_DISCONNECTED TRUE
    CMAKE_ARGS
      -DBUILD_SHARED_LIBS=ON
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/SDL3
      -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
      -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
      -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
      -DSDL_UNIX_CONSOLE_BUILD=ON
      -DSDL_AUDIO=OFF
      -DSDL_CAMERA=OFF
      -DSDL_EXAMPLES=OFF
      -DSDL_GPU=OFF
      -DSDL_RENDER=OFF
      -DSDL_TEST_LIBRARY=OFF
      -DSDL_TESTS=OFF
      -DSDL_VIDEO=OFF
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/SDL3"
  )

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/SDL3/
    DESTINATION ${CMAKE_INSTALL_PREFIX})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
