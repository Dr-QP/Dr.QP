//
// Created by Anton Matosov on 6/16/17.
//

#pragma once 

#include "SerialProtocol.h"

class SerialDecorator : public SerialProtocol
{
public:
    SerialDecorator(SerialProtocol &decorated);

    void begin(const unsigned long baudRate, const uint8_t transferConfig) override;
    size_t write(uint8_t byte) override;
    bool available() override;
    uint8_t peek() override;
    uint8_t read() override;

private:
    SerialProtocol &_decorated;
};


