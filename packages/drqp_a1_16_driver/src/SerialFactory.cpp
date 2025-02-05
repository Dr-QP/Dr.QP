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

#include "drqp_a1_16_driver/SerialFactory.h"

#include <drqp_serial/TcpSerial.h>
#include <drqp_serial/UnixSerial.h>

std::unique_ptr<SerialProtocol> makeSerialForDevice(std::string deviceAddress)
{
  std::unique_ptr<SerialProtocol> servoSerial;
  if (deviceAddress.at(0) == '/') {
    servoSerial = std::make_unique<UnixSerial>(deviceAddress);
    servoSerial->begin(115200);
  } else {
    std::string port = "2022";
    if (auto pos = deviceAddress.find(':'); pos != std::string::npos) {
      port = deviceAddress.substr(pos + 1);
      deviceAddress.erase(pos);
    }
    servoSerial = std::make_unique<TcpSerial>(deviceAddress, port);
  }
  return servoSerial;
}
