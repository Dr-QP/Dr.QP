//
// Created by Anton Matosov on 6/16/17.
//

#include "SerialRecordingProxy.h"

SerialRecordingProxy::SerialRecordingProxy(const std::string& filename)
    : _file(filename, std::ios_base::out | std::ios_base::trunc)
{

}

void SerialRecordingProxy::begin(const unsigned long baudRate, const uint8_t transferConfig)
{

}

size_t SerialRecordingProxy::write(uint8_t byte)
{
    _file << byte;
    return 1;
}

bool SerialRecordingProxy::available()
{
    return false;
}

uint8_t SerialRecordingProxy::peek()
{
    return kNoData;
}

uint8_t SerialRecordingProxy::read()
{
    return kNoData;
}
