//
// Created by Anton Matosov on 6/16/17.
//

#include "drqp_serial/SerialDecorator.h"
#include <assert.h>

SerialDecorator::SerialDecorator(SerialProtocol &decorated)
    : decorated_(decorated)
{
}

void SerialDecorator::begin(const unsigned long baudRate, const uint8_t transferConfig)
{
    return decorated_.begin(baudRate, transferConfig);
}

size_t SerialDecorator::write(uint8_t byte)
{
    return decorated_.write(byte);
}

bool SerialDecorator::available()
{
    return decorated_.available();
}

uint8_t SerialDecorator::peek()
{
    return decorated_.peek();
}

uint8_t SerialDecorator::read()
{
    return decorated_.read();
}
