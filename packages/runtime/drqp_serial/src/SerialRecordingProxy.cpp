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

#include "drqp_serial/SerialRecordingProxy.h"

#include <drqp_rapidjson/ostreamwrapper.h>
#include <drqp_rapidjson/writer.h>

#include <fstream>

namespace RecordingProxy
{
SerialRecordingProxy::SerialRecordingProxy(
  DecoratedPtr decorated, const std::filesystem::path& filename)
: SerialDecorator(std::move(decorated)), fileName_(filename)
{
}

SerialRecordingProxy::~SerialRecordingProxy()
{
  save();
}

void SerialRecordingProxy::begin(const uint32_t baudRate, const uint8_t transferConfig)
{
  super::begin(baudRate, transferConfig);
}

void SerialRecordingProxy::save()
{
  startNewRecordIfDirty();  // flush current record

  std::ofstream ofs(fileName_, std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
  rapidjson::OStreamWrapper osw(ofs);

  rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);

  writer.StartObject();
  {
    writer.Key("records");
    {
      writer.StartArray();
      for (const auto& record : records_) {
        RecordingProxy::write(writer, record);
      }
      writer.EndArray();
    }
  }
  writer.EndObject();
}

bool SerialRecordingProxy::available()
{
  return super::available();
}

size_t SerialRecordingProxy::writeBytes(const void* buffer, size_t size)
{
  assert(buffer);
  const uint8_t* data = static_cast<const uint8_t*>(buffer);

  startNewRecordIfRead();
  size_t result = super::writeBytes(data, size);
  currentRecord_.request.bytes.insert(currentRecord_.request.bytes.end(), data, data + size);

  lastOperation_ = OperationType::kWrite;
  return result;
}

void SerialRecordingProxy::startNewRecordIfRead()
{
  if (lastOperation_ == OperationType::kRead) {
    startNewRecordIfDirty();
  }
}

void SerialRecordingProxy::startNewRecordIfDirty()
{
  if (lastOperation_ != OperationType::kUndefined) {
    records_.push_back(currentRecord_);

    currentRecord_ = Record();
    lastOperation_ = OperationType::kUndefined;
  }
}

size_t SerialRecordingProxy::readBytes(void* buffer, size_t size)
{
  assert(buffer);
  uint8_t* data = static_cast<uint8_t*>(buffer);

  size_t result = super::readBytes(data, size);
  currentRecord_.response.bytes.insert(currentRecord_.response.bytes.end(), data, data + size);

  lastOperation_ = OperationType::kRead;
  return result;
}
}  // namespace RecordingProxy
