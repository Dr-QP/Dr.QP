
add_library(servo STATIC
    include/servo/SerialProtocol.h
    include/servo/Stream.h
    include/servo/TcpSerial.h
    include/servo/UnixSerial.h
    include/servo/XYZrobotServo.h

    src/TcpSerial.cpp
    src/UnixSerial.cpp
    src/XYZrobotServo.cpp
)
find_package(Boost REQUIRED system)

target_include_directories(servo PUBLIC include)
target_link_libraries(servo PUBLIC Boost::system)

