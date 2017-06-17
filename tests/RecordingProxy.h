//
// Created by Anton Matosov on 6/17/17.
//

#pragma once

#include <vector>
#include <boost/serialization/vector.hpp>

namespace RecordingProxy
{
struct Request
{
    std::vector<uint8_t> bytes;
    size_t writeMatchPosition;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & bytes;
    }
};

struct Response
{
    std::vector<uint8_t> bytes;
    size_t readMatchPosition;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & bytes;
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
};
};


