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

#include <cassert>
#include <fstream>

#include <boost/archive/text_iarchive.hpp>

namespace RecordingProxy
{
void SerialPlayer::begin(const uint32_t baudRate, const uint8_t transferConfig) {}

size_t SerialPlayer::write(const uint8_t* data, size_t size)
{
  size_t result = 0;
  assert(data);
  while (size) {
    result += write(*data);

    --size;
    ++data;
  }
  return result;
}

size_t SerialPlayer::write(uint8_t byte)
{
  Record& current = currentRecord();
  if (assertEqual) {
    assertEqual(current.request.bytes.front(), byte);
  }
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

size_t SerialPlayer::readBytes(uint8_t* buffer, size_t size)
{
  assert(buffer);

  const size_t result = size;
  while (size) {
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

  // "records"
  const auto& records = doc["records"].GetArray();
  for (const auto& record : records) {
    Record r;
    RecordingProxy::read(record, r);
    records_.push_back(r);
  }
}
}  // namespace RecordingProxy
