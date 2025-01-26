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

#include <boost/thread.hpp>

#include <catch_ros2/catch.hpp>

#include "drqp_serial/SerialPlayer.h"
#include "drqp_serial/SerialRecordingProxy.h"
#include "drqp_serial/UnixSerial.h"

void simpleSerialTest(SerialProtocol& serial)
{
  serial.begin(115200);

  REQUIRE(serial.write('a') == 1);
  REQUIRE(serial.write('b') == 1);
  REQUIRE(serial.write('c') == 1);
  REQUIRE(serial.write('d') == 1);
  REQUIRE(serial.write('e') == 1);
  REQUIRE(serial.write('f') == 1);
  REQUIRE(serial.write('g') == 1);
  REQUIRE(serial.write('\n') == 1);

  std::string read;
  uint8_t lastRead = 0;
  while (lastRead != '\n') {
    if (serial.available()) {
      lastRead = serial.read();
      read += lastRead;
    } else {
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
  }
  REQUIRE(read == "hello\n");
}

SCENARIO("test unix serial with serial proxy")
{
  static const char* const kSerialRecordingFileName = "tests/test_data/serial_recording.txt";

  WHEN("recording does not exists")
  {
    THEN("record it")
    {
      //            UnixSerial unixSerial("/dev/ttys000");
      ////            UnixSerial unixSerial("/dev/cu.SLAB_USBtoUART");
      //
      //            RecordingProxy::SerialRecordingProxy serial(unixSerial,
      //            kSerialRecordingFileName); simpleSerialTest(serial);
    }
  }

  WHEN("recording exists")
  {
    RecordingProxy::SerialPlayer serialPlayer;
    serialPlayer.assertEqual = [](const uint8_t expected, const uint8_t actual) {
      REQUIRE(expected == actual);
    };

    serialPlayer.load(kSerialRecordingFileName);

    THEN("same test should give same results")
    {
      simpleSerialTest(serialPlayer);
    }
  }
}
