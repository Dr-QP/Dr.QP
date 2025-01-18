cmake_minimum_required(VERSION 3.8)
project(drqp_serial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system)

add_library(drqp_serial STATIC
    include/drqp_serial/RecordingProxy.h
    include/drqp_serial/SerialDecorator.h
    include/drqp_serial/SerialPlayer.h
    include/drqp_serial/SerialProtocol.h
    include/drqp_serial/SerialRecordingProxy.h
    include/drqp_serial/Stream.h
    include/drqp_serial/TcpSerial.h
    include/drqp_serial/UnixSerial.h

    src/RecordingProxy.cpp
    src/SerialDecorator.cpp
    src/SerialPlayer.cpp
    src/SerialRecordingProxy.cpp
    src/TcpSerial.cpp
    src/UnixSerial.cpp
)

target_include_directories(drqp_serial PUBLIC include)
target_link_libraries(drqp_serial PUBLIC Boost::system)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(drqp_serial PRIVATE Threads::Threads $<$<PLATFORM_ID:Linux>:rt>)


if(BUILD_TESTING)
  find_package(catch_ros2 REQUIRED)
  add_executable(drqp_serial_test
    test/drqp_serial_test.cpp
  )

  # https://docs.ros.org/en/humble/p/catch_ros2/index.html
  target_link_libraries(drqp_serial_test
    catch_ros2::catch_ros2_with_main
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
