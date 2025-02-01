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

#include <assert.h>

#include <cstddef>
#include <deque>
#include <functional>
#include <string>

#include "drqp_serial/RecordingProxy.h"
#include "drqp_serial/SerialProtocol.h"

namespace RecordingProxy
{
class SerialPlayer : public SerialProtocol
{
public:
  void begin(const uint32_t baudRate, const uint8_t transferConfig) override;
  size_t write(uint8_t byte) override;
  bool available() override;
  uint8_t peek() override;
  uint8_t read() override;
  void flushRead() override;
  size_t write(const void* data, size_t size) override;
  size_t readBytes(void* buffer, size_t size) override;

  void load(const std::string& fileName);

  using AssertEqual = std::function<void(const uint8_t expected, const uint8_t actual)>;
  AssertEqual assertEqual = [](const uint8_t expected, const uint8_t actual) {
    assert(expected == actual);
  };

private:
  Record currentRecord_;
  OperationType lastOperation_;
  std::deque<Record> records_;
  Record& currentRecord();
};
}  // namespace RecordingProxy
