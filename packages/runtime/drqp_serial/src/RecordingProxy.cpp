// Copyright (c) 2017-present Anton Matosov
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

#include "drqp_serial/RecordingProxy.h"

#include <iostream>

namespace RecordingProxy
{
void write(rapidjson::Writer<rapidjson::OStreamWrapper>& writer, const Packet& packet)
{
  writer.StartObject();
  {
    writer.Key("bytes");
    {
      writer.StartArray();
      for (const auto byte : packet.bytes) {
        writer.Uint(byte);
      }
      writer.EndArray();
    }
  }
  writer.EndObject();
}

void read(const rapidjson::Value& value, Packet& packet)
{
  packet.bytes.clear();
  for (const auto& byte : value["bytes"].GetArray()) {
    packet.bytes.push_back(byte.GetUint());
  }
}

void write(rapidjson::Writer<rapidjson::OStreamWrapper>& writer, const Record& record)
{
  writer.StartObject();
  {
    writer.Key("request");
    write(writer, record.request);

    writer.Key("response");
    write(writer, record.response);
  }
  writer.EndObject();
}

void read(const rapidjson::Value& value, Record& record)
{
  read(value["request"], record.request);
  read(value["response"], record.response);
}
}  // namespace RecordingProxy
