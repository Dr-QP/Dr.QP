//
// Created by Anton Matosov on 6/15/17.
//

#include "UnixSerial.h"
#include "SerialRecordingProxy.h"
//#include <boost/asio.hpp>
#include <boost/thread.hpp>


//#define BOOST_TEST_MODULE MoreSerialTests

#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(testSerialPort)
{
    UnixSerial unixSerial("/dev/ttys002");
//    UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");

    RecordingProxy::SerialRecordingProxy recordingSerial(unixSerial, "serialRecording.txt");
    recordingSerial.begin(115200);

    recordingSerial.write('a');
    recordingSerial.write('b');
    recordingSerial.write('c');
    recordingSerial.write('d');
    recordingSerial.write('e');
    recordingSerial.write('f');
    recordingSerial.write('g');
    recordingSerial.write('\n');

    uint8_t lastRead = 0;
    while (lastRead != '\n')
    {
        if (recordingSerial.available())
        {
            lastRead = recordingSerial.read();
            std::cout << lastRead;
            std::cout.flush();
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

}

