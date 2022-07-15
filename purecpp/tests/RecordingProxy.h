//
// Created by Anton Matosov on 6/17/17.
//

#pragma once

#include <vector>
#include <deque>
#include <boost/serialization/deque.hpp>

namespace RecordingProxy
{
enum class OperationType
{
    kUndefined,
    kRead,
    kWrite
};

struct Request
{
    std::deque<uint8_t> bytes;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & bytes;
    }

    bool empty() const
    {
        return bytes.empty();
    }
};

struct Response
{
    std::deque<uint8_t> bytes;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & bytes;
    }

    bool empty() const
    {
        return bytes.empty();
    }
};

struct Record
{
    Request request;
    Response response;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & request;
        ar & response;
    }

    bool empty() const
    {
        return request.empty() && response.empty();
    }
};
};


