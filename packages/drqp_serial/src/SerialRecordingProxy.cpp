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

#include "drqp_serial/SerialRecordingProxy.h"

#include <boost/archive/text_oarchive.hpp>

namespace RecordingProxy {

SerialRecordingProxy::SerialRecordingProxy(SerialProtocol& decorated, const std::string& filename)
  : SerialDecorator(decorated), lastOperation_(OperationType::kUndefined), fileName_(filename) {}

SerialRecordingProxy::~SerialRecordingProxy() {
  save();
}

void SerialRecordingProxy::begin(const uint32_t baudRate, const uint8_t transferConfig) {
  super::begin(baudRate, transferConfig);
}

size_t SerialRecordingProxy::write(uint8_t byte) {
  startNewRecordIfNeeded();
  size_t result = super::write(byte);
  currentRecord_.request.bytes.push_back(byte);

  lastOperation_ = OperationType::kWrite;
  return result;
}

void SerialRecordingProxy::startNewRecordIfNeeded() {
  if (lastOperation_ == OperationType::kRead) {
    records_.push_back(currentRecord_);

    currentRecord_ = Record();
  }
}

void SerialRecordingProxy::save() {
  startNewRecordIfNeeded();  // flush current record

  std::ofstream file(fileName_, std::ios_base::out | std::ios_base::binary | std::ios_base::trunc);
  boost::archive::text_oarchive archive(file);

  archive & records_;
}

bool SerialRecordingProxy::available() {
  return super::available();
}

uint8_t SerialRecordingProxy::peek() {
  return super::peek();
}

uint8_t SerialRecordingProxy::read() {
  auto result = super::read();
  currentRecord_.response.bytes.push_back(result);

  lastOperation_ = OperationType::kRead;

  return result;
}
}  // namespace RecordingProxy
