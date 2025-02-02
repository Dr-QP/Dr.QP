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

void torqueOff(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  SJogCommand<kServoCount> sposCmd = {
    0,
    {
      SJogData{0, SET_TORQUE_OFF, kTestServo},
      SJogData{0, SET_TORQUE_OFF, kTestServoOther},
    }
  };

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
    }
  };

  servo.sendJogCommand(sposCmd);
}

void neutralPose(SerialProtocol& servoSerial)
{
  XYZrobotServo servo(servoSerial, XYZrobotServo::kBroadcastId);
  constexpr uint16_t kGoal = 512;
  IJogCommand<kServoCount> posCmd = {
    {
      IJogData{kGoal, SET_POSITION_CONTROL, kTestServo, 30},
      IJogData{kGoal, SET_POSITION_CONTROL, kTestServoOther, 30},
    }
  };

  servo.sendJogCommand(posCmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  XYZrobotServo testServo(servoSerial, kTestServo);
  XYZrobotServo testServoOther(servoSerial, kTestServoOther);

  auto status1 = testServo.readStatus();
  REQUIRE_THAT(status1.position, Catch::Matchers::WithinAbs(kGoal, 5));

  auto status2 = testServoOther.readStatus();
  REQUIRE_THAT(status2.position, Catch::Matchers::WithinAbs(kGoal, 5));
}

bool testOptionUseRealHardware = true;
bool testOptionResetTorque = false;
bool testOptionReturnToNeutral = true;

SCENARIO("A1-16 servo operations")
{
  static std::filesystem::path kSerialRecordingFile =
    std::filesystem::current_path() / "test_data" / "a1-16_servo_operations.json";

  GIVEN("A serial")
  {
    UnixSerial serial("/dev/ttySC0");
    serial.begin(115200);
    if (testOptionUseRealHardware)
    {
      if (testOptionResetTorque)
      {
        torqueOff(serial);
        torqueOn(serial);
      }

      if (testOptionReturnToNeutral)
      {
        neutralPose(serial);
      }
    }

    RecordingProxy::SerialRecordingProxy recorder(serial, kSerialRecordingFile);

    WHEN("action")
    {
      THEN("response") {}
    }
  }
}
