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

#include <memory>
#include <thread>
#include <chrono>
#include <filesystem>

#include <catch_ros2/catch.hpp>

#include "drqp_serial/SerialPlayer.h"
#include "drqp_serial/SerialRecordingProxy.h"
#include "drqp_serial/UnixSerial.h"

void simpleSerialTest(SerialProtocol& serial)
{
  serial.begin(115200);

  const std::string str = "abcdefg\n";
  const size_t writtenSize = serial.writeBytes(str.c_str(), str.size());
  REQUIRE(writtenSize == str.size());

  std::string read;
  uint8_t lastRead = 0;
  while (lastRead != '\n') {
    if (serial.available()) {
      serial.readBytes(&lastRead, 1);
      read += lastRead;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  REQUIRE(read == "hello\n");
}

SCENARIO("test unix serial with serial proxy")
{
  static std::filesystem::path kSourceSerialRecordingFile =
    std::filesystem::path(TEST_DATA_DIR_IN_SOURCE_TREE) / "serial_recording.json";
  static std::filesystem::path kDestinationSerialRecordingFile =
    std::filesystem::path(TEST_DATA_DIR_IN_BUILD_TREE) / "destination_serial_recording.json";

  WHEN("destination recording does not exist")
  {
    REQUIRE(exists(kSourceSerialRecordingFile));
    REQUIRE(!exists(kDestinationSerialRecordingFile));

    THEN("record it from source recording")
    {
      // The source serial for raw recording
      // UnixSerial sourceSerial("/dev/ttySC0");

      auto sourceSerial = std::make_unique<RecordingProxy::SerialPlayer>();
      sourceSerial->load(kSourceSerialRecordingFile);

      RecordingProxy::SerialRecordingProxy serial(
        std::move(sourceSerial), kDestinationSerialRecordingFile);
      simpleSerialTest(serial);
    }
  }

  WHEN("destination recording exists")
  {
    REQUIRE(exists(kDestinationSerialRecordingFile));
    RecordingProxy::SerialPlayer serialPlayer;
    serialPlayer.assertEqual = [](const uint8_t expected, const uint8_t actual, const size_t pos) {
      INFO("Comparing position " << pos);
      REQUIRE(expected == actual);
    };

    serialPlayer.load(kDestinationSerialRecordingFile);

    THEN("same test should give same results")
    {
      simpleSerialTest(serialPlayer);

      REQUIRE_NOTHROW(remove(kDestinationSerialRecordingFile));
    }
  }
}
