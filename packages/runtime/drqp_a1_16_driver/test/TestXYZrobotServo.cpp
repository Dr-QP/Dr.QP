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

#include "drqp_serial/SerialProtocol.h"
#include "drqp_serial/SerialFactory.h"
#include "drqp_a1_16_driver/XYZrobotServo.h"

#include "drqp_serial/SerialPlayer.h"
#include "drqp_serial/SerialRecordingProxy.h"

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
  std::string serialAddress = "/dev/ttySC0";
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
  cli |= Opt(
    testOptions.serialAddress, "device")["--device"]("UART device or IP:PORT for ser2net device");

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

void neutralPoseIJog(SerialProtocol& servoSerial)
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

void neutralPoseDynamicIJog(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);

  std::vector<IJogData> posCmd(kServoCount);
  posCmd.at(0) = IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServo, 30};
  posCmd.at(1) = IJogData{kStartGoal, SET_POSITION_CONTROL, kTestServoOther, 30};
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

void neutralPoseDynamicSJog(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);

  DynamicSJogCommand sposCmd(kServoCount);
  sposCmd.setPlaytime(200);
  sposCmd.at(0) = SJogData{kStartGoal, SET_POSITION_CONTROL, kTestServo};
  sposCmd.at(1) = SJogData{kStartGoal, SET_POSITION_CONTROL, kTestServoOther};
  servo.sendJogCommand(sposCmd);

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
    std::unique_ptr<SerialProtocol> serial = makeSerialForDevice(testOptions.serialAddress);
    serial->begin(115200);
    if (testOptions.resetTorque) {
      torqueOff(*serial);
      torqueOn(*serial);
    }

    if (!testOptions.skipReturnToNeutral) {
      neutralPoseIJog(*serial);
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

  XYZrobotServoRAM ram;
  servo.ramRead(0, &ram, sizeof(ram));
  REQUIRE(servo.isOk());

  REQUIRE(ram.sID == kTestServo);
  REQUIRE_THAT(ram.Joint_Position, GoalPositionWithin(kStartGoal));
  REQUIRE_THAT(ram.Position_Goal, GoalPositionWithin(kStartGoal));
  REQUIRE_THAT(ram.Position_Ref, GoalPositionWithin(kStartGoal));
}

// This is a bad test, because it will write back the RAM and might brick the servo
// TEST_CASE("A1-16 servo read all RAM and write back")
// {
//   std::unique_ptr<SerialProtocol> serial = makeSerial("read-ram-write-back");
//   XYZrobotServo servo(*serial, kTestServo);

//   XYZrobotServoRAM ram;
//   servo.ramRead(0, &ram, sizeof(ram));

//   REQUIRE(servo.isOk());

//   servo.ramWrite(0, &ram, sizeof(ram));
//   REQUIRE(servo.isOk());
// }

TEST_CASE("A1-16 servo read all EEPROM")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("read-eeprom");
  XYZrobotServo servo(*serial, kTestServo);

  XYZrobotServoEEPROM eeprom;
  servo.eepromRead(0, &eeprom, sizeof(eeprom));

  REQUIRE(servo.isOk());

  REQUIRE(eeprom.Model_Number == 0x01);   // Model number
  REQUIRE(eeprom.Year == 0x0F);           // Year
  REQUIRE(eeprom.Version_Month == 0x3A);  // Version/Month
  REQUIRE(eeprom.Day == 0x01);            // Day

  REQUIRE(eeprom.sID == kTestServo);
  REQUIRE(eeprom.Min_Position == 23);
  REQUIRE(eeprom.Max_Position == 1000);
}

// This is a bad test, because it will write back the EEPROM and brick the servo
// TEST_CASE("A1-16 servo read all EEPROM and write back")
// {
//   std::unique_ptr<SerialProtocol> serial = makeSerial("read-eeprom-write-back");
//   XYZrobotServo servo(*serial, kTestServo);

//   XYZrobotServoEEPROM eeprom;
//   servo.eepromRead(0, &eeprom, sizeof(eeprom));
//   REQUIRE(servo.isOk());

//   servo.eepromWrite(0, &eeprom, sizeof(eeprom));
//   REQUIRE(servo.isOk());
// }

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

  neutralPoseIJog(*serial);
}

TEST_CASE("A1-16 servo set dynamic I-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-dynamic-i-jog");

  neutralPoseDynamicIJog(*serial);
}

TEST_CASE("A1-16 servo set S-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-s-jog");

  neutralPoseSJog(*serial);
}

TEST_CASE("A1-16 servo set dynamic S-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-neutral-pose-dynamic-s-jog");

  neutralPoseDynamicSJog(*serial);
}

TEST_CASE("A1-16 servo set torque off and back on via S-JOG")
{
  std::unique_ptr<SerialProtocol> serial = makeSerial("set-torque-off-and-on-via-s-jog");

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

TEST_CASE("A1-16 servo max PWM RAM")
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

static_assert(sizeof(XYZrobotServoRAM) == 80, "XYZrobotServoRAM has wrong size");

static_assert(offsetof(XYZrobotServoRAM, sID) == 0, "XYZrobotServoRAM::sID has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, ACK_Policy) == 1, "XYZrobotServoRAM::ACK_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Alarm_LED_Policy) == 2,
  "XYZrobotServoRAM::Alarm_LED_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Torque_Policy) == 3,
  "XYZrobotServoRAM::Torque_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, SPDctrl_Policy) == 4,
  "XYZrobotServoRAM::SPDctrl_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Temperature) == 5,
  "XYZrobotServoRAM::Max_Temperature has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Min_Voltage) == 6, "XYZrobotServoRAM::Min_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Voltage) == 7, "XYZrobotServoRAM::Max_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Acceleration_Ratio) == 8,
  "XYZrobotServoRAM::Acceleration_Ratio has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Wheel_Ref_Position) == 12,
  "XYZrobotServoRAM::Max_Wheel_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_PWM) == 16, "XYZrobotServoRAM::Max_PWM has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Overload_Threshold) == 18,
  "XYZrobotServoRAM::Overload_Threshold has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Min_Position) == 20,
  "XYZrobotServoRAM::Min_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Max_Position) == 22,
  "XYZrobotServoRAM::Max_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Kp) == 24, "XYZrobotServoRAM::Position_Kp has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Kd) == 26, "XYZrobotServoRAM::Position_Kd has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Ki) == 28, "XYZrobotServoRAM::Position_Ki has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Close_to_Open_Ref_Position) == 30,
  "XYZrobotServoRAM::Close_to_Open_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Open_to_Close_Ref_Position) == 32,
  "XYZrobotServoRAM::Open_to_Close_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Ramp_Speed) == 36, "XYZrobotServoRAM::Ramp_Speed has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, LED_Blink_Period) == 38,
  "XYZrobotServoRAM::LED_Blink_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Packet_Timeout_Detection_Period) == 40,
  "XYZrobotServoRAM::Packet_Timeout_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Overload_Detection_Period) == 42,
  "XYZrobotServoRAM::Overload_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Inposition_Margin) == 44,
  "XYZrobotServoRAM::Inposition_Margin has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Over_Voltage_Detection_Period) == 45,
  "XYZrobotServoRAM::Over_Voltage_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Over_Temperature_Detection_Period) == 46,
  "XYZrobotServoRAM::Over_Temperature_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Calibration_Difference) == 47,
  "XYZrobotServoRAM::Calibration_Difference has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Status_Error) == 48,
  "XYZrobotServoRAM::Status_Error has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Status_Detail) == 49,
  "XYZrobotServoRAM::Status_Detail has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, LED_Control) == 53, "XYZrobotServoRAM::LED_Control has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Voltage) == 54, "XYZrobotServoRAM::Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Temperature) == 55, "XYZrobotServoRAM::Temperature has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Current_Control_Mode) == 56,
  "XYZrobotServoRAM::Current_Control_Mode has wrong offset");
static_assert(offsetof(XYZrobotServoRAM, Tick) == 57, "XYZrobotServoRAM::Tick has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Joint_Position) == 60,
  "XYZrobotServoRAM::Joint_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, PWM_Output_Duty) == 64,
  "XYZrobotServoRAM::PWM_Output_Duty has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Bus_Current) == 66, "XYZrobotServoRAM::Bus_Current has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Goal) == 68,
  "XYZrobotServoRAM::Position_Goal has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Position_Ref) == 70,
  "XYZrobotServoRAM::Position_Ref has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Omega_Goal) == 72, "XYZrobotServoRAM::Omega_Goal has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Omega_Ref) == 74, "XYZrobotServoRAM::Omega_Ref has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, Requested_Counts) == 76,
  "XYZrobotServoRAM::Requested_Counts has wrong offset");
static_assert(
  offsetof(XYZrobotServoRAM, ACK_Counts) == 78, "XYZrobotServoRAM::ACK_Counts has wrong offset");

static_assert(sizeof(XYZrobotServoEEPROM) == 54, "XYZrobotServoEEPROM has wrong size");

static_assert(
  offsetof(XYZrobotServoEEPROM, Model_Number) == 0,
  "XYZrobotServoEEPROM::Model_Number has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Year) == 1, "XYZrobotServoEEPROM::Year has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Version_Month) == 2,
  "XYZrobotServoEEPROM::Version_Month has wrong offset");
static_assert(offsetof(XYZrobotServoEEPROM, Day) == 3, "XYZrobotServoEEPROM::Day has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Baud_Rate) == 5, "XYZrobotServoEEPROM::Baud_Rate has wrong offset");
static_assert(offsetof(XYZrobotServoEEPROM, sID) == 6, "XYZrobotServoEEPROM::sID has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, ACK_Policy) == 7,
  "XYZrobotServoEEPROM::ACK_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Alarm_LED_Policy) == 8,
  "XYZrobotServoEEPROM::Alarm_LED_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Torque_Policy) == 9,
  "XYZrobotServoEEPROM::Torque_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, SPDctrl_Policy) == 10,
  "XYZrobotServoEEPROM::SPDctrl_Policy has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Max_Temperature) == 11,
  "XYZrobotServoEEPROM::Max_Temperature has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Min_Voltage) == 12,
  "XYZrobotServoEEPROM::Min_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Max_Voltage) == 13,
  "XYZrobotServoEEPROM::Max_Voltage has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Acceleration_Ratio) == 14,
  "XYZrobotServoEEPROM::Acceleration_Ratio has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved1) == 15,
  "XYZrobotServoEEPROM::reserved1 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Max_Wheel_Ref_Position) == 18,
  "XYZrobotServoEEPROM::Max_Wheel_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved2) == 20,
  "XYZrobotServoEEPROM::reserved2 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Max_PWM) == 22, "XYZrobotServoEEPROM::Max_PWM has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Overload_Threshold) == 24,
  "XYZrobotServoEEPROM::Overload_Threshold has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Min_Position) == 26,
  "XYZrobotServoEEPROM::Min_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Max_Position) == 28,
  "XYZrobotServoEEPROM::Max_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Position_Kp) == 30,
  "XYZrobotServoEEPROM::Position_Kp has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Position_Kd) == 32,
  "XYZrobotServoEEPROM::Position_Kd has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Position_Ki) == 34,
  "XYZrobotServoEEPROM::Position_Ki has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Close_to_Open_Ref_Position) == 36,
  "XYZrobotServoEEPROM::Close_to_Open_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Open_to_Close_Ref_Position) == 38,
  "XYZrobotServoEEPROM::Open_to_Close_Ref_Position has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved3) == 40,
  "XYZrobotServoEEPROM::reserved3 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Ramp_Speed) == 42,
  "XYZrobotServoEEPROM::Ramp_Speed has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, LED_Blink_Period) == 44,
  "XYZrobotServoEEPROM::LED_Blink_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved4) == 45,
  "XYZrobotServoEEPROM::reserved4 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Packet_Timeout_Detection_Period) == 46,
  "XYZrobotServoEEPROM::Packet_Timeout_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved5) == 47,
  "XYZrobotServoEEPROM::reserved5 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Overload_Detection_Period) == 48,
  "XYZrobotServoEEPROM::Overload_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, reserved6) == 49,
  "XYZrobotServoEEPROM::reserved6 has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Inposition_Margin) == 50,
  "XYZrobotServoEEPROM::Inposition_Margin has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Over_Voltage_Detection_Period) == 51,
  "XYZrobotServoEEPROM::Over_Voltage_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Over_Temperature_Detection_Period) == 52,
  "XYZrobotServoEEPROM::Over_Temperature_Detection_Period has wrong offset");
static_assert(
  offsetof(XYZrobotServoEEPROM, Calibration_Difference) == 53,
  "XYZrobotServoEEPROM::Calibration_Difference has wrong offset");

