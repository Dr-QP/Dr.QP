// Copyright (c) 2017-2025 Anton Matosov
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#pragma once

#include <drqp_rapidjson/writer.h>
#include <drqp_rapidjson/document.h>
#include <drqp_rapidjson/ostreamwrapper.h>

#include <deque>

#include <boost/serialization/deque.hpp>
#include <boost/serialization/split_member.hpp>

namespace RecordingProxy
{
enum class OperationType { kUndefined, kRead, kWrite };

struct Packet
{
  std::deque<uint8_t> bytes;
};

void write(rapidjson::Writer<rapidjson::OStreamWrapper>& writer, const Packet& packet);
void read(const rapidjson::Value& value, Packet& packet);

struct Record
{
  BOOST_SERIALIZATION_SPLIT_MEMBER();

  Packet request;
  Packet response;

  bool empty() const
  {
    return request.bytes.empty() && response.bytes.empty();
  }
};

void write(rapidjson::Writer<rapidjson::OStreamWrapper>& writer, const Record& record);
void read(const rapidjson::Value& value, Record& record);
}  // namespace RecordingProxy
