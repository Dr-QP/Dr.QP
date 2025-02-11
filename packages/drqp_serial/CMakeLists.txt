cmake_minimum_required(VERSION 3.30..3.31)
project(drqp_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

include(../cmake/PatchDepends.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)

ament_package_xml()
patch_depends_variables()

foreach(dep ${${PROJECT_NAME}_BUILD_DEPENDS})
  if (dep STREQUAL "boost")
    set(dep Boost)
  endif()

  find_package(${dep} REQUIRED)
endforeach()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(drqp_serial STATIC
    src/RecordingProxy.cpp
    src/SerialDecorator.cpp
    src/SerialPlayer.cpp
    src/SerialRecordingProxy.cpp
    src/TcpSerial.cpp
    src/UnixSerial.cpp
)

target_compile_features(drqp_serial PUBLIC cxx_std_17)
target_include_directories(drqp_serial PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(drqp_serial PUBLIC ${${PROJECT_NAME}_BUILD_DEPENDS})

target_link_libraries(drqp_serial PRIVATE Threads::Threads $<$<PLATFORM_ID:Linux>:rt>)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Install libraries
install(TARGETS
  drqp_serial
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(${${PROJECT_NAME}_BUILD_EXPORT_DEPENDS})


if(BUILD_TESTING)
  include(../cmake/ClangFormatConfig.cmake)
  include(../cmake/Catch2Extras.cmake)
  add_subdirectory(test)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
