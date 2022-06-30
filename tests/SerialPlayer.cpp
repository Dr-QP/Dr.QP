//
// Created by Anton Matosov on 6/15/17.
//

#include "SerialPlayer.h"
#include <cassert>
#include <fstream>
#include <catch2/catch_test_macros.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace RecordingProxy
{
void SerialPlayer::begin(const unsigned long baudRate, const uint8_t transferConfig)
{

}
size_t SerialPlayer::write(const uint8_t *data, size_t size)
{
    size_t result = 0;
    assert(data);
    while (size)
    {
        result += write(*data);

        --size;
        ++data;
    }
    return result;
}

size_t SerialPlayer::write(uint8_t byte)
{
    Record& current = currentRecord();
    REQUIRE(current.request.bytes.front() == byte);
    current.request.bytes.pop_front();
    return 1;
}

bool SerialPlayer::available()
{
    return currentRecord().response.bytes.size() > 0;
}

uint8_t SerialPlayer::peek()
{
    return currentRecord().response.bytes.front();
}

size_t SerialPlayer::readBytes(uint8_t *buffer, size_t size)
{
    assert(buffer);

    const size_t result = size;
    while (size)
    {
        *buffer = read();

        --size;
        ++buffer;
    }
    return result;
}

uint8_t SerialPlayer::read()
{
    auto val = currentRecord().response.bytes.front();
    currentRecord_.response.bytes.pop_front();
    return val;
}

void SerialPlayer::flushRead()
{
}

Record &SerialPlayer::currentRecord()
{
    if (currentRecord_.empty()) {
        REQUIRE(records_.size() != 0);

        currentRecord_ = records_.front();
        records_.pop_front();
    }
    return currentRecord_;
}

void SerialPlayer::load(const std::string& fileName)
{
    std::ifstream file(fileName);
    boost::archive::text_iarchive archive(file);

    archive & records_;
}
}