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
#include <filesystem>
#include <functional>
#include <string>

#include "drqp_serial/RecordingProxy.h"
#include "drqp_serial/SerialProtocol.h"

namespace RecordingProxy
{
class SerialPlayer : public SerialProtocol
{
public:
  SerialPlayer() = default;
  explicit SerialPlayer(const std::filesystem::path& fileName);
  ~SerialPlayer();

  void begin(const uint32_t baudRate, const uint8_t transferConfig) override;
  bool available() override;

  void flushRead() override;

  size_t writeBytes(const void* buffer, size_t size) override;
  size_t readBytes(void* buffer, size_t size) override;

  void load(const std::filesystem::path& fileName);

  using AssertEqual =
    std::function<void(const uint8_t expected, const uint8_t actual, const size_t pos)>;
  AssertEqual assertEqual = [](const uint8_t expected, const uint8_t actual, const size_t pos) {
    if (expected != actual) {
      throw std::runtime_error(
        "Expected " + std::to_string(expected) + " but got " + std::to_string(actual) +
        " at position " + std::to_string(pos));
    }
  };

  using BeforeDestructionCallback = std::function<void(RecordingProxy::SerialPlayer& player)>;
  void beforeDestruction(BeforeDestructionCallback callback);

  bool isEmpty() const;

private:
  Record currentRecord_;
  OperationType lastOperation_;
  std::deque<Record> records_;
  BeforeDestructionCallback beforeDestructionCallback_;
  Record& currentRecord();
};
}  // namespace RecordingProxy
