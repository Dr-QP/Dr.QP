//
// Created by Anton Matosov on 6/16/17.
//

#include "drqp_serial/SerialRecordingProxy.h"
#include <boost/archive/text_oarchive.hpp>

namespace RecordingProxy
{

SerialRecordingProxy::SerialRecordingProxy(SerialProtocol &decorated, const std::string &filename)
    : SerialDecorator(decorated)
    , fileName_(filename)
    , lastOperation_(OperationType::kUndefined)
{

}

SerialRecordingProxy::~SerialRecordingProxy()
{
    save();
}

void SerialRecordingProxy::begin(const unsigned long baudRate, const uint8_t transferConfig)
{
    super::begin(baudRate, transferConfig);
}

size_t SerialRecordingProxy::write(uint8_t byte)
{
    startNewRecordIfNeeded();
    size_t result = super::write(byte);
    currentRecord_.request.bytes.push_back(byte);

    lastOperation_ = OperationType::kWrite;
    return result;
}

void SerialRecordingProxy::startNewRecordIfNeeded()
{
    if (lastOperation_ == OperationType::kRead) {
        records_.push_back(currentRecord_);

        currentRecord_ = Record();
    }
}

void SerialRecordingProxy::save()
{
    startNewRecordIfNeeded(); // flush current record

    std::ofstream file(fileName_, std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
    boost::archive::text_oarchive archive(file);

    archive & records_;
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
    auto result = super::read();
    currentRecord_.response.bytes.push_back(result);

    lastOperation_ = OperationType::kRead;

    return result;
}
}
