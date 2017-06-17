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

//    using namespace boost::asio;
//    io_service portService;
//    serial_port serial(portService, "/dev/cu.SLAB_USBtoUART");
//
//    serial.set_option(serial_port_base::baud_rate(115200));
//
//    std::string s = "abcdefg\n";
//    boost::asio::async_write(serial, boost::asio::buffer(s.c_str(), s.size()), transfer_all());
//
//
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

int main()
{
    testSerialPort();

    return 0;
}
