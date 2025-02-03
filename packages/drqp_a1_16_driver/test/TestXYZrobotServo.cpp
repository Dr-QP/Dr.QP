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

/// Global test options
// TODO(anton-matosov): Add command line options for these options
struct TestOptions
{
  bool useRealHardware = true;
  bool resetTorque = false;
  bool returnToNeutral = true;
};
static TestOptions testOptions;

void waitHardwareForMilliseconds(uint64_t milliseconds)
{
  if (testOptions.useRealHardware) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
  }
}

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
  waitHardwareForMilliseconds(500);
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

Catch::Matchers::WithinAbsMatcher GoalPositionWithin(uint16_t goalPosition)
{
  return Catch::Matchers::WithinAbs(goalPosition, 5);
}

uint16_t waitForPosition(XYZrobotServo& testServo, uint16_t goalPosition)
{
  uint16_t currentPosition = 0;

  int retries = 3;
  do {
    waitHardwareForMilliseconds(300);
    currentPosition = testServo.readStatus().position;
  } while (!GoalPositionWithin(goalPosition).match(currentPosition) && --retries > 0);

  INFO(
    "Exceeded number of retries (" << retries << ") while waiting to reach the goalPosition="
                                   << goalPosition << ". CurrentPosition=" << currentPosition);
  REQUIRE(retries > 0);

  return currentPosition;
}

void neutralPose(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  IJogCommand<kServoCount> posCmd = {{
    IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServo, 30},
    IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServoOther, 30},
  }};

  servo.sendJogCommand(posCmd);

  XYZrobotServo testServo(servoSerial, kTestServo);
  XYZrobotServo testServoOther(servoSerial, kTestServoOther);

  REQUIRE_THAT(waitForPosition(testServo, kStartGoal), GoalPositionWithin(kStartGoal));
  REQUIRE_THAT(waitForPosition(testServoOther, kStartGoal), GoalPositionWithin(kStartGoal));
}

void neutralPoseSJog(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  SJogCommand<kServoCount> posCmd = {
    200,  // Give it enough time,
    {
      SJogData{kStartGoal, SET_POSITION_CONTROL, kTestServo},
      SJogData{kStartGoal, SET_POSITION_CONTROL, kTestServoOther},
    }};

  servo.sendJogCommand(posCmd);

  XYZrobotServo testServo(servoSerial, kTestServo);
  XYZrobotServo testServoOther(servoSerial, kTestServoOther);

  REQUIRE_THAT(waitForPosition(testServo, kStartGoal), GoalPositionWithin(kStartGoal));
  REQUIRE_THAT(waitForPosition(testServoOther, kStartGoal), GoalPositionWithin(kStartGoal));
}

std::filesystem::path makeRecordingName(const std::string& suffix)
{
  static const std::filesystem::path basePath = TEST_DATA_DIR_IN_SOURCE_TREE;
  REQUIRE(exists(basePath));
  auto result = basePath / ("a1-16_servo-" + suffix + ".json");
  if (!testOptions.useRealHardware) {
    INFO("When running in simulation mode, the test recording file should exist. Path: " << result);
    REQUIRE(exists(result));
  }
  return result;
}

std::unique_ptr<SerialProtocol> makeSerial(const std::string& suffix)
{
  const auto filename = makeRecordingName(suffix);

  if (testOptions.useRealHardware) {
    std::unique_ptr<UnixSerial> serial = std::make_unique<UnixSerial>("/dev/ttySC0");
    serial->begin(115200);
    if (testOptions.resetTorque) {
      torqueOff(*serial);
      torqueOn(*serial);
    }

    if (testOptions.returnToNeutral) {
      neutralPose(*serial);
    }

    return std::make_unique<RecordingProxy::SerialRecordingProxy>(std::move(serial), filename);
  } else {
    std::unique_ptr<RecordingProxy::SerialPlayer> serialPlayer =
      std::make_unique<RecordingProxy::SerialPlayer>();

    serialPlayer->load(filename);

    serialPlayer->beforeDestruction([](RecordingProxy::SerialPlayer& player) {
      INFO("Verify that all records have been accessed by the test");
      REQUIRE(player.isEmpty());
    });

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
        REQUIRE_THAT(jointPosition, GoalPositionWithin(kStartGoal));

        uint16_t positionGoal = ram[68] + (ram[69] << 8);
        REQUIRE_THAT(positionGoal, GoalPositionWithin(kStartGoal));

        uint16_t positionRef = ram[60] + (ram[71] << 8);
        REQUIRE_THAT(positionRef, GoalPositionWithin(kStartGoal));
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

  WHEN("set position")
  {
    std::unique_ptr<SerialProtocol> serial = makeSerial("set-position");
    XYZrobotServo servo(*serial, kTestServo);

    servo.setPosition(kTestGoal, 10);

    REQUIRE_THAT(waitForPosition(servo, kTestGoal), GoalPositionWithin(kTestGoal));
  }

  WHEN("set I-JOG")
  {
    std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-i-jog");

    neutralPose(*serial);
  }

  WHEN("set S-JOG")
  {
    std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-s-jog");

    neutralPoseSJog(*serial);
  }

  WHEN("set torque off and back on via S-JOG")
  {
    std::unique_ptr<SerialProtocol> serial = makeSerial("set-torque-off-and-on-via-sjog");

    torqueOff(*serial);

    torqueOn(*serial);
  }

  WHEN("reboot and back on")
  {
    std::unique_ptr<SerialProtocol> serial = makeSerial("reboot-and-torque-back-on");

    XYZrobotServo servo(*serial, kTestServo);
    servo.reboot();

    INFO("waiting for reboot to finish");
    waitHardwareForMilliseconds(3000);

    INFO("turning torque back on");
    servo.torqueOn();
  }
}
