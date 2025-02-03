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

#define CATCH_CONFIG_RUNNER  // For custom main
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
  bool useRealHardware = false;
  bool resetTorque = false;
  bool skipReturnToNeutral = false;
};
static TestOptions testOptions;

int main(int argc, char* argv[])
{
  Catch::Session session;

  using Catch::Clara::Opt;
  auto cli = session.cli();
  cli |= Opt(testOptions.useRealHardware)["--use-real-hardware"]("Use real hardware servos");
  cli |= Opt(testOptions.resetTorque)["--reset-torque"]("reset torque");
  cli |= Opt(testOptions.skipReturnToNeutral)["--no-neutral"]("Skip return to neutral");

  session.cli(cli);

  auto ret = session.applyCommandLine(argc, argv);
  if (ret) {
    return ret;
  }

  return session.run();
}

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

    if (!testOptions.skipReturnToNeutral) {
      neutralPose(*serial);
    }

    return std::make_unique<RecordingProxy::SerialRecordingProxy>(std::move(serial), filename);
  } else {
    std::unique_ptr<RecordingProxy::SerialPlayer> serialPlayer =
      std::make_unique<RecordingProxy::SerialPlayer>();
    serialPlayer->assertEqual = [](const uint8_t expected, const uint8_t actual, const size_t pos) {
      INFO("Comparing position " << pos);
      REQUIRE(expected == actual);
    };

    serialPlayer->load(filename);

    serialPlayer->beforeDestruction([](RecordingProxy::SerialPlayer& player) {
      INFO("Verify that all records have been accessed by the test");
      REQUIRE(player.isEmpty());
    });

    return serialPlayer;
  }
}

TEST_CASE("A1-16 servo read all RAM")
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

TEST_CASE("A1-16 servo read all EEPROM")
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

TEST_CASE("A1-16 servo set position")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-position");
  XYZrobotServo servo(*serial, kTestServo);

  servo.setPosition(kTestGoal, 10);

  REQUIRE_THAT(waitForPosition(servo, kTestGoal), GoalPositionWithin(kTestGoal));
}

TEST_CASE("A1-16 servo set I-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-i-jog");

  neutralPose(*serial);
}

TEST_CASE("A1-16 servo set S-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-s-jog");

  neutralPoseSJog(*serial);
}

TEST_CASE("A1-16 servo set torque off and back on via S-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-torque-off-and-on-via-sjog");

  torqueOff(*serial);

  torqueOn(*serial);
}

TEST_CASE("A1-16 servo reboot and back on")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("reboot-and-torque-back-on");

  XYZrobotServo servo(*serial, kTestServo);
  servo.reboot();

  INFO("waiting for reboot to finish");
  waitHardwareForMilliseconds(3000);

  INFO("turning torque back on");
  servo.torqueOn();
}

TEST_CASE("A1-16 servo write led policy")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("write-led-policy");

  XYZrobotServo servo(*serial, kTestServo);

  INFO("Make while LED controllable by user");
  servo.writeAlarmLedPolicyRam(XYZrobotServo::kWhiteLedBit);
  REQUIRE(servo.isOk());

  INFO("Turn off all user LEDs");
  servo.writeLedControl(0);
  REQUIRE(servo.isOk());

  uint8_t ledControl = {};
  servo.ramRead(53, &ledControl, sizeof(ledControl));
  REQUIRE(servo.isOk());
  REQUIRE(ledControl == 0);

  waitHardwareForMilliseconds(20);
  INFO("Restore all LEDs to be controllable by system");
  servo.writeLedControl(0xF);
  waitHardwareForMilliseconds(20);

  servo.writeAlarmLedPolicyRam(0);
  REQUIRE(servo.isOk());
}

TEST_CASE("A1-16 servo read ack policy ram")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("read-ack-policy-ram");

  XYZrobotServo servo(*serial, kTestServo);

  XYZrobotServoAckPolicy ackPolicy = servo.readAckPolicyRam();
  REQUIRE(servo.isOk());

  REQUIRE(ackPolicy == XYZrobotServoAckPolicy::OnlyReadAndStat);
}

TEST_CASE("A1-16 servo max PWM RAM", "[focus]")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("max-pwm-ram");

  XYZrobotServo servo(*serial, kTestServo);

  servo.writeMaxPwmRam(5);
  REQUIRE(servo.isOk());
  waitHardwareForMilliseconds(20);

  servo.setPosition(kTestGoal, 10);

  REQUIRE_THAT(waitForPosition(servo, kTestGoal), GoalPositionWithin(kTestGoal));

  constexpr uint16_t defaultPWM = 1023;
  servo.writeMaxPwmRam(defaultPWM);
  REQUIRE(servo.isOk());
  waitHardwareForMilliseconds(20);
}

// Next test stub
// TEST_CASE("A1-16 servo ", "[focus]")
// {
// }
