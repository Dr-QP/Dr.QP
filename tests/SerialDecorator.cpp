//
// Created by Anton Matosov on 6/16/17.
//

#include "SerialDecorator.h"
#include <assert.h>

SerialDecorator::SerialDecorator(SerialProtocol &decorated)
    : _decorated(decorated)
{
}

void SerialDecorator::begin(const unsigned long baudRate, const uint8_t transferConfig)
{
    return _decorated.begin(baudRate, transferConfig);
}

size_t SerialDecorator::write(uint8_t byte)
{
    return _decorated.write(byte);
}

bool SerialDecorator::available()
{
    return _decorated.available();
}

uint8_t SerialDecorator::peek()
{
    return _decorated.peek();
}

uint8_t SerialDecorator::read()
{
    return _decorated.read();
}
