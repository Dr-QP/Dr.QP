//
// Created by Anton Matosov on 6/15/17.
//

#include "UnixSerial.h"
//#include <boost/asio.hpp>
#include <boost/thread.hpp>

void testSerialPort()
{
    UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");
    unixSerial.begin(115200);

    unixSerial.write('a');
    unixSerial.write('b');
    unixSerial.write('c');
    unixSerial.write('d');
    unixSerial.write('e');
    unixSerial.write('f');
    unixSerial.write('g');
    unixSerial.write('\n');

    uint8_t lastRead = 0;
    while (lastRead != '\n')
    {
        if (unixSerial.available())
        {
            lastRead = unixSerial.read();
            std::cout << lastRead;
            std::cout.flush();
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

}

int main()
{
    testSerialPort();

    return 0;
}
