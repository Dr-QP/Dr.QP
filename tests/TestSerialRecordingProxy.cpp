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

    serial.write('a');
    serial.write('b');
    serial.write('c');
    serial.write('d');
    serial.write('e');
    serial.write('f');
    serial.write('g');
    serial.write('\n');

    uint8_t lastRead = 0;
    while (lastRead != '\n')
    {
        if (serial.available())
        {
            lastRead = serial.read();
            std::cout << lastRead;
            std::cout.flush();
        }
        else
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
    }
}

BOOST_AUTO_TEST_CASE(testSerialPortRecord)
{
    UnixSerial unixSerial("/dev/ttys002");
//    UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");

    RecordingProxy::SerialRecordingProxy serial(unixSerial, kSerialRecordingFileName);
    simpleSerialTest(serial);
}

BOOST_AUTO_TEST_CASE(testSerialPortPlay)
{
    RecordingProxy::SerialPlayer serial;
    serial.load(kSerialRecordingFileName);
    simpleSerialTest(serial);
}
