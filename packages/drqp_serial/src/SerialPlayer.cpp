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

#include "drqp_serial/SerialPlayer.h"

#include <drqp_rapidjson/document.h>
#include <drqp_rapidjson/istreamwrapper.h>

#include <algorithm>
#include <cassert>
#include <fstream>

namespace RecordingProxy
{
void SerialPlayer::begin(const uint32_t baudRate, const uint8_t transferConfig) {}

size_t SerialPlayer::writeBytes(const void* buffer, size_t size)
{
  assert(buffer);
  assert(assertEqual);
  const uint8_t* data = static_cast<const uint8_t*>(buffer);

  Record& record = currentRecord();

  if (size > record.request.bytes.size()) {
    // If requested write is larger than was recorded, its a fail
    assertEqual(size, record.request.bytes.size(), 0);
  }

  for (size_t i = 0; i < size; ++i) {
    // Compare what was written now with recording
    assertEqual(data[i], record.request.bytes[i], i);
  }

  // Remove verified recording
  // TODO(anton-matosov): Consider keeping a lastWritePosition instead of erasing.
  //  Or use range and update it every write
  record.request.bytes.erase(record.request.bytes.begin(), record.request.bytes.begin() + size);

  return size;
}

bool SerialPlayer::available()
{
  return currentRecord().response.bytes.size() > 0;
}

size_t SerialPlayer::readBytes(void* buffer, size_t size)
{
  assert(buffer);
  uint8_t* data = static_cast<uint8_t*>(buffer);

  Record& record = currentRecord();
  const size_t availableSize = std::min(size, record.response.bytes.size());

  std::copy_n(record.response.bytes.begin(), availableSize, data);
  record.response.bytes.erase(
    record.response.bytes.begin(), record.response.bytes.begin() + availableSize);

  return availableSize;
}

void SerialPlayer::flushRead() {}

Record& SerialPlayer::currentRecord()
{
  if (currentRecord_.empty()) {
    assert(records_.size() != 0);

    currentRecord_ = records_.front();
    records_.pop_front();
  }
  return currentRecord_;
}

void SerialPlayer::load(const std::string& fileName)
{
  std::ifstream file(fileName);

  rapidjson::IStreamWrapper inputStream(file);

  rapidjson::Document doc;
  doc.ParseStream(inputStream);

  const auto& records = doc["records"].GetArray();
  for (const auto& record : records) {
    Record r;
    RecordingProxy::read(record, r);
    records_.push_back(r);
  }
}
}  // namespace RecordingProxy
