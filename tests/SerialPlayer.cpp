//
// Created by Anton Matosov on 6/15/17.
//

#include "SerialPlayer.h"
#include <boost/test/unit_test.hpp>

namespace RecordingProxy
{
void SerialPlayer::begin(const unsigned long baudRate, const uint8_t transferConfig)
{

}

size_t SerialPlayer::write(uint8_t byte)
{
    Record current = currentRecord();
    BOOST_TEST(current.request.bytes.front() == byte);
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

uint8_t SerialPlayer::read()
{
    auto val = currentRecord().response.bytes.front();
    currentRecord_.response.bytes.pop_front();
    return val;
}

Record &SerialPlayer::currentRecord()
{
    if (currentRecord_.empty()) {
        BOOST_TEST(records_.size() != 0);

        currentRecord_ = records_.front();
        records_.pop_front();
    }
    return currentRecord_;
}
}