#pragma once

#include <inttypes.h>
#include <stddef.h>

class Stream
{
public:
    virtual bool available() = 0;
    virtual void flushRead() = 0;
    virtual uint8_t peek() = 0;
    virtual uint8_t read() = 0;
    virtual size_t write(uint8_t byte) = 0;
    virtual size_t write(const uint8_t *data, size_t size) = 0;
    virtual size_t readBytes(uint8_t *buffer, size_t size) = 0;
    

    virtual ~Stream()
    { }
};