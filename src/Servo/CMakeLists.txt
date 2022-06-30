
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

