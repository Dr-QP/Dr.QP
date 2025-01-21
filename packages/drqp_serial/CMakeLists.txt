cmake_minimum_required(VERSION 3.8)
project(drqp_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

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
target_link_libraries(drqp_serial PUBLIC Boost::boost) # Link to all of boost, its the only way ament can find it in downstream packages

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
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
ament_export_dependencies(Boost)



if(BUILD_TESTING)
  find_package(catch_ros2 REQUIRED)
  add_executable(drqp_serial_test
    test/drqp_serial_test.cpp
  )

  # https://docs.ros.org/en/humble/p/catch_ros2/index.html
  target_link_libraries(drqp_serial_test PRIVATE
    catch_ros2::catch_ros2_with_main
    drqp_serial
  )



  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
