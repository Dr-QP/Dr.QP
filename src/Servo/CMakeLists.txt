cmake_minimum_required(VERSION 3.11)

project(Servo CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Werror)

add_library(LibServo STATIC
    SerialProtocol.h
    Stream.h
    TcpSerial.h
    UnixSerial.h
    XYZrobotServo.h

    TcpSerial.cpp
    UnixSerial.cpp
    XYZrobotServo.cpp
)
find_package(Boost REQUIRED system)
target_link_libraries(LibServo PUBLIC Boost::boost ${Boost_LIBRARIES})

add_executable(Servo ReadEverything.cpp)
add_executable(Relax Relax.cpp)

target_link_libraries(Servo PRIVATE LibServo)
target_link_libraries(Relax PRIVATE LibServo)
