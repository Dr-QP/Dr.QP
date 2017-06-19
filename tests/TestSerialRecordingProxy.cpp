//
// Created by Anton Matosov on 6/19/17.
//

#include <boost/test/unit_test.hpp>

#include "UnixSerial.h"
#include "SerialPlayer.h"
#include "SerialRecordingProxy.h"
#include <boost/thread.hpp>


static const char *const kSerialRecordingFileName = "serialRecording.txt";

void simpleSerialTest(SerialProtocol &serial)
{
    serial.begin(115200);

    BOOST_TEST(serial.write('a') == 1);
    BOOST_TEST(serial.write('b') == 1);
    BOOST_TEST(serial.write('c') == 1);
    BOOST_TEST(serial.write('d') == 1);
    BOOST_TEST(serial.write('e') == 1);
    BOOST_TEST(serial.write('f') == 1);
    BOOST_TEST(serial.write('g') == 1);
    BOOST_TEST(serial.write('\n') == 1);

    std::stringstream read;
    uint8_t lastRead = 0;
    while (lastRead != '\n')
    {
        if (serial.available())
        {
            lastRead = serial.read();
            read << lastRead;
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
    }
    BOOST_TEST(read.str() == "hello\n");
}
//
//BOOST_AUTO_TEST_CASE(testSerialPortRecord)
//{
//    UnixSerial unixSerial("/dev/ttys002");
////    UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");
//
//    RecordingProxy::SerialRecordingProxy serial(unixSerial, kSerialRecordingFileName);
//    simpleSerialTest(serial);
//}

BOOST_AUTO_TEST_CASE(testSerialPlayback)
{
    RecordingProxy::SerialPlayer serial;
    serial.load(kSerialRecordingFileName);
    simpleSerialTest(serial);
}
