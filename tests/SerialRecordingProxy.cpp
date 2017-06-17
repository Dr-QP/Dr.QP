//
// Created by Anton Matosov on 6/16/17.
//

#include "SerialRecordingProxy.h"

namespace RecordingProxy
{

SerialRecordingProxy::SerialRecordingProxy(SerialProtocol &decorated, const std::string &filename)
    : SerialDecorator(decorated)
    , _file(filename, std::ios_base::out | std::ios_base::trunc)
{

}

void SerialRecordingProxy::begin(const unsigned long baudRate, const uint8_t transferConfig)
{
    super::begin(baudRate, transferConfig);
}

size_t SerialRecordingProxy::write(uint8_t byte)
{
    size_t result = super::write(byte);
    _file << byte;
    _file << result;
    return result;
}

bool SerialRecordingProxy::available()
{
    return super::available();
}

uint8_t SerialRecordingProxy::peek()
{
    return super::peek();
}

uint8_t SerialRecordingProxy::read()
{
    return super::read();
}
}
