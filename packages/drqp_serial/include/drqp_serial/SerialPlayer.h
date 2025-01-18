//
// Created by Anton Matosov on 6/15/17.
//

#pragma once 

#include <cstddef>
#include "drqp_serial/SerialProtocol.h"
#include <deque>
#include "RecordingProxy.h"

namespace RecordingProxy
{
class SerialPlayer: public SerialProtocol
{
public:
    void begin(const unsigned long baudRate, const uint8_t transferConfig) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    uint8_t peek() override;
    uint8_t read() override;
    void flushRead() override;
    size_t write(const uint8_t *data, size_t size) override;
    size_t readBytes(uint8_t *buffer, size_t size) override;

    void load(const std::string &fileName);
private:
    Record currentRecord_;
    OperationType lastOperation_;
    std::deque<Record> records_;
    Record &currentRecord();
};
}

