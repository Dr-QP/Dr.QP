//
// Created by Anton Matosov on 6/19/17.
//

#include <catch.hpp>

#include "UnixSerial.h"
#include "SerialPlayer.h"
#include "SerialRecordingProxy.h"
#include <boost/thread.hpp>

void simpleSerialTest(SerialProtocol &serial)
{
    serial.begin(115200);

    REQUIRE(serial.write('a') == 1);
    REQUIRE(serial.write('b') == 1);
    REQUIRE(serial.write('c') == 1);
    REQUIRE(serial.write('d') == 1);
    REQUIRE(serial.write('e') == 1);
    REQUIRE(serial.write('f') == 1);
    REQUIRE(serial.write('g') == 1);
    REQUIRE(serial.write('\n') == 1);

    std::stringstream read;
    uint8_t lastRead = 0;
    while (lastRead != '\n') {
        if (serial.available()) {
            lastRead = serial.read();
            read << lastRead;
        } else {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
    }
    REQUIRE(read.str() == "hello\n");
}

SCENARIO("test unix serial with serial proxy", "[serial_proxy], [unix_serial]")
{
    static const char *const kSerialRecordingFileName = "test_data/serial_recording.txt";

    WHEN("recording does not exists") {
        THEN("record it") {
//            UnixSerial unixSerial("/dev/ttys000");
////            UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");
//
//            RecordingProxy::SerialRecordingProxy serial(unixSerial, kSerialRecordingFileName);
//            simpleSerialTest(serial);
        }
    }

    WHEN("recording exists") {
        RecordingProxy::SerialPlayer serial;

        serial.load(kSerialRecordingFileName);

        THEN("same test should give same results") {
            simpleSerialTest(serial);
        }
    }
}