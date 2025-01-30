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

#include <thread>
#include "drqp_control/DrQp.h"

#include "drqp_a1_16_driver/XYZrobotServo.h"
#include "drqp_serial/TcpSerial.h"
#include "drqp_serial/UnixSerial.h"

void forEachServo(long millisecondsBetweenLegs, std::function<void(ServoId servoId, int servoIndexInLeg)> func) {
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

int main(const int argc, const char* const argv[])
{
  // TcpSerial servoSerial("192.168.1.136", 2022);
  UnixSerial servoSerial("/dev/ttySC0");

  const Pose kPoseSet = [argc, argv](){
    if (argc >= 2) {
      std::string pose = argv[1];
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
    }
    return kNeutralPose;
  }();

  forEachServo(200, [&kPoseSet, &servoSerial](ServoId servoId, int servoIndexInLeg) {
    XYZrobotServo servo(servoSerial, servoId);

    const ServoPosition position = kPoseSet[kServoIdToLeg[servoId]][servoIndexInLeg];
    servo.setPosition(position, 100);
  });

  return 0;
}
