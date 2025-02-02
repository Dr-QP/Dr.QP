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

#include <cstdint>
#include <thread>
#include <chrono>
#include <filesystem>

#include <catch_ros2/catch.hpp>

#include "drqp_a1_16_driver/XYZrobotServo.h"

#include "drqp_serial/SerialPlayer.h"
#include "drqp_serial/SerialRecordingProxy.h"
#include "drqp_serial/UnixSerial.h"

using ServoId = uint8_t;
constexpr uint8_t kServoCount = 2;
using ServoIdsArray = std::array<ServoId, kServoCount>;

constexpr ServoId kTestServo = 5;
constexpr ServoId kTestServoOther = 3;

ServoIdsArray kTestServoIds = {kTestServo, kTestServoOther};

constexpr uint16_t kStartGoal = 512;
constexpr uint16_t kTestGoal = 812;

void torqueOff(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  SJogCommand<kServoCount> sposCmd = {
    0,
    {
      SJogData{0, SET_TORQUE_OFF, kTestServo},
      SJogData{0, SET_TORQUE_OFF, kTestServoOther},
    }};

  servo.sendJogCommand(sposCmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void torqueOn(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  SJogCommand<kServoCount> sposCmd = {
    0,
    {
      SJogData{0, SET_POSITION_CONTROL_SERVO_ON, kTestServo},
      SJogData{0, SET_POSITION_CONTROL_SERVO_ON, kTestServoOther},
    }};

  servo.sendJogCommand(sposCmd);
}

void neutralPose(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  IJogCommand<kServoCount> posCmd = {{
    IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServo, 30},
    IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServoOther, 30},
  }};

  servo.sendJogCommand(posCmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  XYZrobotServo testServo(servoSerial, kTestServo);
  XYZrobotServo testServoOther(servoSerial, kTestServoOther);

  auto status1 = testServo.readStatus();
  REQUIRE_THAT(status1.position, Catch::Matchers::WithinAbs(kStartGoal, 5));

  auto status2 = testServoOther.readStatus();
  REQUIRE_THAT(status2.position, Catch::Matchers::WithinAbs(kStartGoal, 5));
}

static bool testOptionUseRealHardware = false;
static bool testOptionResetTorque = false;
static bool testOptionReturnToNeutral = true;

std::filesystem::path makeRecordingName(const std::string& suffix)
{
  static const std::filesystem::path basePath = TEST_DATA_DIR_IN_SOURCE_TREE;
  REQUIRE(exists(basePath));
  auto result = basePath / ("a1-16_servo-" + suffix + ".json");
  if (!testOptionUseRealHardware) {
    INFO("When running in simulation mode, the test recording file should exist. Path: " << result);
    REQUIRE(exists(result));
  }
  return result;
}

std::unique_ptr<SerialProtocol> makeSerial(const std::string& suffix)
{
  const auto filename = makeRecordingName(suffix);

  if (testOptionUseRealHardware) {
    std::unique_ptr<UnixSerial> serial = std::make_unique<UnixSerial>("/dev/ttySC0");
    serial->begin(115200);
    if (testOptionResetTorque) {
      torqueOff(*serial);
      torqueOn(*serial);
    }

    if (testOptionReturnToNeutral) {
      neutralPose(*serial);
    }

    return std::make_unique<RecordingProxy::SerialRecordingProxy>(std::move(serial), filename);
  } else {
    std::unique_ptr<RecordingProxy::SerialPlayer> serialPlayer =
      std::make_unique<RecordingProxy::SerialPlayer>();

    serialPlayer->load(filename);

    return serialPlayer;
  }
}

SCENARIO("A1-16 servo operations")
{
  GIVEN("A serial")
  {
    WHEN("read all RAM")
    {
      std::unique_ptr<SerialProtocol> serial = makeSerial("read-ram");
      XYZrobotServo servo(*serial, kTestServo);

      uint8_t ram[80] = {};

      servo.ramRead(0, ram, 30);
      REQUIRE(servo.isOk());

      servo.ramRead(30, ram + 30, 30);
      REQUIRE(servo.isOk());

      servo.ramRead(60, ram + 60, 20);
      REQUIRE(servo.isOk());

      THEN("should have correct values")
      {
        REQUIRE(ram[0] == kTestServo);

        uint16_t jointPosition = ram[60] + (ram[61] << 8);
        REQUIRE_THAT(jointPosition, Catch::Matchers::WithinAbs(kStartGoal, 5));
      }
    }

    WHEN("read all EEPROM")
    {
      std::unique_ptr<SerialProtocol> serial = makeSerial("read-eeprom");
      XYZrobotServo servo(*serial, kTestServo);

      uint8_t eeprom[54];
      servo.eepromRead(0, eeprom, 30);
      REQUIRE(servo.isOk());

      servo.eepromRead(30, eeprom + 30, 24);
      REQUIRE(servo.isOk());

      THEN("should have correct values")
      {
        REQUIRE(eeprom[0] == 0x01);  // Model number
        REQUIRE(eeprom[1] == 0x0F);  // Year
        REQUIRE(eeprom[2] == 0x3A);  // Version/Month
        REQUIRE(eeprom[3] == 0x01);  // Day
        REQUIRE(eeprom[6] == kTestServo);

        uint16_t minPosition = eeprom[26] + (eeprom[27] << 8);
        uint16_t maxPosition = eeprom[28] + (eeprom[29] << 8);
        REQUIRE(minPosition == 23);
        REQUIRE(maxPosition == 1000);
      }
    }
  }
}
