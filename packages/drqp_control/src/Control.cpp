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

#include <drqp_serial/SerialProtocol.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <thread>
#include "drqp_control/DrQp.h"

#include "drqp_a1_16_driver/XYZrobotServo.h"
#include "drqp_a1_16_driver/SerialFactory.h"

void forEachServo(
  uint64_t millisecondsBetweenLegs, std::function<void(ServoId servoId, int servoIndexInLeg)> func)
{
  if (!func) {
    return;
  }

  for (const auto& leg : kAllLegServoIds) {
    int servoIndexInLeg = 0;
    for (const int servoId : leg) {
      func(servoId, servoIndexInLeg);

      ++servoIndexInLeg;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(millisecondsBetweenLegs));
  }
}

void readAll(SerialProtocol& servoSerial)
{
  forEachServo(0, [&servoSerial](ServoId servoId, int) {
    XYZrobotServo servo(servoSerial, servoId);

    XYZrobotServoStatus status = servo.readStatus();
    if (servo.isFailed()) {
      std::cerr << legNameForServo(servoId) << " servo: " << static_cast<int>(servoId)
                << " error reading status: " << servo.getLastError() << "\n";
    } else {
      std::cout << legNameForServo(servoId) << "\tservo: " << static_cast<int>(servoId) << ": "
                << status.position << "\t posRef: " << status.posRef
                << "\t statusError: " << status.statusError << "\n";
    }
  });
}

int main(const int argc, const char* const argv[])
{
  try {
    const std::string pose = (argc >= 2 ? argv[1] : "neutral");
    const std::string deviceAddress = (argc >= 3 ? argv[2] : "/dev/ttySC0");

    std::unique_ptr<SerialProtocol> servoSerial = makeSerialForDevice(deviceAddress);

    if (pose == "off" || pose == "relax") {
      forEachServo(0, [&servoSerial](ServoId servoId, int) {
        XYZrobotServo servo(*servoSerial, servoId);
        servo.torqueOff();
        if (servo.isFailed()) {
          throw std::runtime_error("Torque OFF failed: " + to_string(servo.getLastError()));
        }
        servo.reboot();
        if (servo.isFailed()) {
          throw std::runtime_error("Reboot failed: " + to_string(servo.getLastError()));
        }
      });

      return 0;
    } else if (pose == "read") {
      readAll(*servoSerial);

      return 0;
    }

    // Recover each servo to its current position
    forEachServo(0, [&servoSerial](ServoId servoId, int) {
      XYZrobotServo servo(*servoSerial, servoId);

      XYZrobotServoStatus status = servo.readStatus();
      if (servo.isFailed()) {
        throw std::runtime_error("Read status failed: " + to_string(servo.getLastError()));
      }
      if (status.pwm == 0 && abs(status.position - status.posRef) > 15) {
        servo.torqueOn();
        if (servo.isFailed()) {
          throw std::runtime_error("Torque ON failed: " + to_string(servo.getLastError()));
        }
        std::cout << "Recovering " << legNameForServo(servoId)
                  << "\tservo: " << static_cast<int>(servoId) << " to " << status.posRef << "\n";
      }
    });

    const Pose kPoseSet = [&pose]() {
      if (pose == "neutral") {
        return kNeutralPose;
      } else if (pose == "stand") {
        return kStandingPose;
      } else if (pose == "down") {
        return kFoldedDownPose;
      } else if (pose == "up") {
        return kFoldedUpPose;
      } else if (pose == "upc") {
        return kFoldedUpCompactPose;
      }

      return kNeutralPose;
    }();

    IJogCommand<kServoCount> iposCmd;
    SJogCommand<kServoCount> sposCmd;
    sposCmd.playtime = 150;

    size_t servoIndex = 0;
    forEachServo(
      0, [&kPoseSet, &iposCmd, &sposCmd, &servoIndex](ServoId servoId, int servoIndexInLeg) {
        iposCmd.data[servoIndex] = {
          kPoseSet[kServoIdToLeg[servoId]][servoIndexInLeg], SET_POSITION_CONTROL, servoId, 150};
        sposCmd.data[servoIndex] = {
          kPoseSet[kServoIdToLeg[servoId]][servoIndexInLeg], SET_POSITION_CONTROL, servoId};

        servoIndex++;
      });
    XYZrobotServo servo(*servoSerial, XYZrobotServo::kBroadcastId);
    // servo.sendJogCommand(sposCmd);
    servo.sendJogCommand(iposCmd);
    if (servo.isFailed()) {
      throw std::runtime_error("set position failed: " + to_string(servo.getLastError()));
    }
  } catch (std::exception& e) {
    std::cerr << "Control call failed with exception: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "Control call failed with unknown exception.\n";
    return 1;
  }
  return 0;
}
