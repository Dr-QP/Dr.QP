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

#pragma once

#include <cstddef>
#include <memory>

#include "drqp_serial/SerialProtocol.h"

class SerialDecorator : public SerialProtocol
{
public:
  using super = SerialProtocol;
  using DecoratedPtr = std::unique_ptr<super>;

  explicit SerialDecorator(DecoratedPtr decorated);

  using super::begin;
  void begin(const uint32_t baudRate, const uint8_t transferConfig) override;
  bool available() override;
  void flushRead() override;

  size_t writeBytes(const void* buffer, size_t size) override;
  size_t readBytes(void* buffer, size_t size) override;

private:
  DecoratedPtr decorated_;
};
