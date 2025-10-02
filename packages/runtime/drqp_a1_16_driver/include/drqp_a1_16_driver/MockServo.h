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

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "drqp_a1_16_driver/ServoProtocol.h"

class MockServo : public ServoProtocol
{
public:
  explicit MockServo(const uint8_t id) : id_(id) {}
  ~MockServo() = default;

  XYZrobotServoStatus readStatus() override
  {
    assert(id_ != kBroadcastId);
    XYZrobotServoStatus status = {};
    status.position = positions_[id_];
    return status;
  }

  bool isFailed() const override
  {
    return false;
  }

  void reboot() override {}

  XYZrobotServoError getLastError() const override
  {
    return XYZrobotServoError::None;
  }

  void sendJogCommand(const std::vector<IJogData>& commands) override
  {
    for (const auto& cmd : commands) {
      positions_[cmd.id] = cmd.goal;
    }
  }

  void ramRead(uint8_t startAddress, void* data, uint8_t dataSize) override
  {
    uint8_t* dataPtr = static_cast<uint8_t*>(data);
    for (uint8_t i = 0; i < dataSize; ++i) {
      dataPtr[i] = 0xBE;
    }
  }

private:
  uint8_t id_;
  static std::unordered_map<uint8_t, uint16_t> positions_;
};
