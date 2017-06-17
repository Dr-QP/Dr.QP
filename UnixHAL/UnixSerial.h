//
// Created by Anton Matosov on 6/17/17.
//

#pragma once 

#include "SerialProtocol.h"
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

class UnixSerial: public SerialProtocol
{
public:
    UnixSerial(const std::string& fileName);

    void begin(const unsigned long baudRate, const uint8_t transferConfig = SERIAL_8N1) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    uint8_t peek() override;
    uint8_t read() override;

private:
    boost::asio::io_service ioService_;
    boost::asio::serial_port serial_;

    uint8_t lastRead_;
    bool everRead_;
};


