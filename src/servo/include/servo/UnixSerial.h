#pragma once 

#include "servo/SerialProtocol.h"
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

class UnixSerial: public SerialProtocol
{
public:
    UnixSerial(const std::string& fileName);

    using SerialProtocol::begin;
    void begin(const unsigned long baudRate, const uint8_t transferConfig) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    void flushRead() override;
    uint8_t peek() override;
    uint8_t read() override;

    size_t write(const uint8_t *data, size_t size) override;
    size_t readBytes(uint8_t *buffer, size_t size) override;
private:
    boost::asio::io_service ioService_;
    boost::asio::serial_port serial_;

    uint8_t lastRead_;
    bool everRead_;
};


