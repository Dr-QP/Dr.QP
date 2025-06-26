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

#pragma once

#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <algorithm>
#include <optional>

#include "drqp_control/DrQp.h"

struct ServoParams
{
  uint8_t id;
  double ratio = 1.;
};
struct JointParams
{
  std::string jointName;
  double ratio = 1.;
};
static const auto [kJointToServoId, kServoIdToJoint] = []() {
  std::unordered_map<std::string, ServoParams> jointToServoId;
  std::unordered_map<uint8_t, JointParams> servoIdToJoint;

  const std::string kRight = "right";
  const std::array<const char*, kServosPerLeg> kJointNames = {"_coxa", "_femur", "_tibia"};
  for (const auto& leg : kAllLegServoIds) {
    int jointNameIndex = 0;
    for (const uint8_t servoId : leg) {
      const std::string legName = legNameForServo(servoId);
      std::string jointName = "dr_qp/" + legName + kJointNames[jointNameIndex];

      const bool isRight =
        std::find_end(legName.begin(), legName.end(), kRight.begin(), kRight.end()) !=
        legName.end();
      const bool isCoxa = jointNameIndex == 0;
      double ratio = (isRight && !isCoxa) ? -1. : 1.;
      jointToServoId[jointName] = {servoId, ratio};
      servoIdToJoint[servoId] = {jointName, ratio};
      ++jointNameIndex;
    }
  }
  return std::make_tuple(jointToServoId, servoIdToJoint);
}();

struct JointValues
{
  std::string name;
  double position_as_radians;
};

struct ServoValues
{
  uint8_t id;
  uint16_t position;
};

static inline std::optional<ServoValues> jointToServo(const JointValues& joint)
{
  if (kJointToServoId.count(joint.name) == 0) {
    return std::nullopt;
  }

  const ServoParams servoParams = kJointToServoId.at(joint.name);
  const uint16_t position = radiansToPosition(joint.position_as_radians * servoParams.ratio);
  return ServoValues{servoParams.id, position};
}

static inline std::optional<JointValues> servoToJoint(const ServoValues& servo)
{
  if (kServoIdToJoint.count(servo.id) == 0) {
    return std::nullopt;
  }

  const JointParams jointParams = kServoIdToJoint.at(servo.id);
  const double positionAsRadians = positionToRadians(servo.position * jointParams.ratio);
  return JointValues{jointParams.jointName, positionAsRadians};
}

static inline uint8_t millisToPlaytime(const uint16_t millis)
{
  return millis / 10;
}
