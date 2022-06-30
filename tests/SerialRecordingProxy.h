//
// Created by Anton Matosov on 6/16/17.
//

#pragma once 

#include <string>
#include <fstream>
#include <deque>
#include "SerialDecorator.h"
#include "servo/SerialProtocol.h"
#include "RecordingProxy.h"

namespace RecordingProxy
{
// The SerialRecordingProxy works for master-slave UART setup only
// Each communication starts on the master side with write command followed by multiple available/read/peak commands
// This type of communication is similar to HTTP request/response model, except that response is read in multiple calls
class SerialRecordingProxy: public SerialDecorator
{
public:
    typedef SerialDecorator super;
    SerialRecordingProxy(SerialProtocol &decorated, const std::string& filename);
    ~SerialRecordingProxy();

    using super::begin;
    void begin(const unsigned long baudRate, const uint8_t transferConfig) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    uint8_t peek() override;
    uint8_t read() override;

private:
    Record currentRecord_;
    OperationType lastOperation_;
    std::deque<Record> records_;

    std::string fileName_;

    void startNewRecordIfNeeded();
    void save();
};
}
