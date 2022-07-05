
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

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(servo PRIVATE Threads::Threads $<$<PLATFORM_ID:Linux>:rt>)
