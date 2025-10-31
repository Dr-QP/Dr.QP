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

#include <array>
#include <cstdint>
#include <cmath>
#include <string>

using ServoId = uint8_t;
constexpr ServoId kServoIdMax = 253;

enum LegId {
  // Head is at 12 o'clock

  kFirstLegId = 0,
  kFrontLeftLegId = 0,  // 11 o'clock
  kFrontRightLegId,     // 1 o'clock

  kMiddleLeftLegId,   // 9 o'clock
  kMiddleRightLegId,  // 3 o'clock

  kBackLeftLegId,   // 7 o'clock
  kBackRightLegId,  // 5 o'clock

  // Must be last
  kLegIdCount
};

constexpr size_t kServosPerLeg = 3;
constexpr const ServoId kServoMinId = 1;
constexpr const ServoId kServoMaxId = kServoMinId + kServosPerLeg * kLegIdCount;
constexpr const size_t kServoCount = kServoMaxId - kServoMinId;

using ServoIdsArray = std::array<ServoId, kServoCount>;

static inline ServoIdsArray servoIdsRange()
{
  ServoIdsArray result;
  size_t i = 0;
  for (ServoId id = kServoMinId; id < kServoMaxId; ++id, ++i) {
    result[i] = id;
  }
  return result;
}

using LegServoIdsArray = std::array<ServoId, kServosPerLeg>;
using AllLegsServoIdsArray = std::array<LegServoIdsArray, kLegIdCount>;

const AllLegsServoIdsArray kAllLegServoIds = {
  LegServoIdsArray{1, 3, 5},  // left_front
  LegServoIdsArray{2, 4, 6},  // right_front

  LegServoIdsArray{13, 15, 17},  // left_middle
  LegServoIdsArray{14, 16, 18},  // right_middle

  LegServoIdsArray{7, 9, 11},   // left_back
  LegServoIdsArray{8, 10, 12},  // right_back
};

using ServoIdToLegArray = std::array<ServoId, kServoMaxId>;

const ServoIdToLegArray kServoIdToLeg = []() {
  ServoIdToLegArray mappings;

  for (int leg = kFirstLegId; leg < kLegIdCount; ++leg) {
    for (const ServoId id : kAllLegServoIds[leg]) {
      mappings[id] = leg;
    }
  }

  return mappings;
}();

using AllLegsNamesArray = std::array<std::string, kLegIdCount>;
const AllLegsNamesArray kAllLegsNames = []() {
  AllLegsNamesArray legs;
  legs[kFrontLeftLegId] = "left_front";
  legs[kFrontRightLegId] = "right_front";

  legs[kMiddleLeftLegId] = "left_middle";
  legs[kMiddleRightLegId] = "right_middle";

  legs[kBackLeftLegId] = "left_back";
  legs[kBackRightLegId] = "right_back";

  return legs;
}();

static inline std::string legNameForServo(ServoId id)
{
  return kAllLegsNames[kServoIdToLeg[id]];
}

template <typename T2 = double>
inline T2 mapToRange(double x, double in_min, double in_max, T2 out_min, T2 out_max)
{
  return static_cast<T2>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

inline uint16_t radiansToPosition(double angle)
{
  return mapToRange<uint16_t>(angle, -M_PI, M_PI, 0, 1023);
}

// Position => Radians
// 0 => -Pi
// 512 => 0
// 1023 => Pi
inline double positionToRadians(uint16_t position)
{
  return mapToRange(position, 0, 1023, -M_PI, M_PI);
}

static inline uint8_t millisToPlaytime(const uint16_t millis)
{
  return millis / 10;
}

// Leg joins in the straight line
// right_front     servo: 2:       516 // Coxa
// right_front     servo: 4:       554 // Femur
// right_front     servo: 6:       592 // Tibia

// left_front      servo: 1:       517
// left_front      servo: 3:       451
// left_front      servo: 5:       429

//          Left              Right
// Coxa     512               512
// Femur    512 - 61 = 451    512 + 42 = 554
// Tibia    512 - 83 = 429    512 + 80 = 592

using ServoPosition = uint16_t;
using LegServoPositionsArray = std::array<ServoPosition, kServosPerLeg>;
using Pose = std::array<LegServoPositionsArray, kLegIdCount>;

const Pose kNeutralPose = []() {
  Pose legs;
  const uint16_t kBase = 512;

  const uint16_t kCoxaOffset = 0;
  const uint16_t kFemurOffset = 50;
  const uint16_t kTibiaOffset = 80;

  const int16_t kRight = 1;
  const int16_t kLeft = -1;

  legs[kFrontRightLegId] = legs[kMiddleRightLegId] = legs[kBackRightLegId] = {
    kBase + kCoxaOffset, kBase + kRight * kFemurOffset, kBase + kRight * kTibiaOffset};
  legs[kFrontLeftLegId] = legs[kMiddleLeftLegId] = legs[kBackLeftLegId] = {
    kBase + kCoxaOffset, kBase + kLeft * kFemurOffset, kBase + kLeft * kTibiaOffset};
  return legs;
}();

const Pose kStandingPose = []() {
  Pose legs;
  const uint16_t kBase = 512;

  const uint16_t kCoxaOffset = 0;
  const uint16_t kFemurOffset = 0;
  const uint16_t kTibiaOffset = 0;

  const int16_t kRight = 1;
  const int16_t kLeft = -1;

  legs[kFrontRightLegId] = legs[kMiddleRightLegId] = legs[kBackRightLegId] = {
    kBase + kCoxaOffset, kBase + kRight * kFemurOffset, kBase + kRight * kTibiaOffset};
  legs[kFrontLeftLegId] = legs[kMiddleLeftLegId] = legs[kBackLeftLegId] = {
    kBase + kCoxaOffset, kBase + kLeft * kFemurOffset, kBase + kLeft * kTibiaOffset};
  return legs;
}();

const Pose kFoldedDownPose = []() {
  Pose legs;
  const uint16_t kBase = 512;

  const uint16_t kCoxaOffset = 0;
  const uint16_t kFemurOffset = 160;
  const uint16_t kTibiaOffset = 290;

  const int16_t kRight = -1;  // Inverted compared to other poses
  const int16_t kLeft = 1;

  legs[kFrontRightLegId] = legs[kMiddleRightLegId] = legs[kBackRightLegId] = {
    kBase + kCoxaOffset, kBase + kRight * kFemurOffset, kBase + kRight * kTibiaOffset};
  legs[kFrontLeftLegId] = legs[kMiddleLeftLegId] = legs[kBackLeftLegId] = {
    kBase + kCoxaOffset, kBase + kLeft * kFemurOffset, kBase + kLeft * kTibiaOffset};
  return legs;
}();

const Pose kFoldedUpPose = []() {
  Pose legs;
  const uint16_t kBase = 512;

  const uint16_t kCoxaOffset = 0;
  const uint16_t kFemurOffset = 280;
  const uint16_t kTibiaOffset = 220;

  const int16_t kRight = 1;
  const int16_t kLeft = -1;

  legs[kFrontRightLegId] = legs[kMiddleRightLegId] = legs[kBackRightLegId] = {
    kBase + kCoxaOffset, kBase + kRight * kFemurOffset, kBase + kRight * kTibiaOffset};
  legs[kFrontLeftLegId] = legs[kMiddleLeftLegId] = legs[kBackLeftLegId] = {
    kBase + kCoxaOffset, kBase + kLeft * kFemurOffset, kBase + kLeft * kTibiaOffset};
  return legs;
}();

const Pose kFoldedUpCompactPose = []() {
  Pose legs;
  const uint16_t kBase = 512;

  const int16_t kCoxaOffset = -260;
  const int16_t kFemurOffset = 280;
  const int16_t kTibiaOffset = 220;

  const int16_t kRight = 1;
  const int16_t kLeft = -1;

  legs[kFrontRightLegId] = legs[kMiddleRightLegId] = legs[kBackRightLegId] = {
    kBase + kRight * kCoxaOffset, kBase + kRight * kFemurOffset, kBase + kRight * kTibiaOffset};
  legs[kFrontLeftLegId] = legs[kMiddleLeftLegId] = legs[kBackLeftLegId] = {
    kBase + kLeft * kCoxaOffset, kBase + kLeft * kFemurOffset, kBase + kLeft * kTibiaOffset};
  return legs;
}();

using FrontRightLeg = LegServoPositionsArray;
using FrontLeftLeg = LegServoPositionsArray;
using MiddleRightLeg = LegServoPositionsArray;
using MiddleLeftLeg = LegServoPositionsArray;
using BackRightLeg = LegServoPositionsArray;
using BackLeftLeg = LegServoPositionsArray;

const AllLegsNamesArray kLegClassNames = []() {
  AllLegsNamesArray legs;
  legs[kFrontLeftLegId] = "FrontLeftLeg";
  legs[kFrontRightLegId] = "FrontRightLeg";

  legs[kMiddleLeftLegId] = "MiddleLeftLeg";
  legs[kMiddleRightLegId] = "MiddleRightLeg";

  legs[kBackLeftLegId] = "BackLeftLeg";
  legs[kBackRightLegId] = "BackRightLeg";

  return legs;
}();

// clang-format off
static const Pose kStandingNarrowPose = {
  FrontLeftLeg{642, 322, 840}, FrontRightLeg{382, 702, 184},
  MiddleLeftLeg{512, 322, 840}, MiddleRightLeg{512, 702, 184},
  BackLeftLeg{382, 322, 840}, BackRightLeg{642, 702, 184},
};

// left middle, right front and back up
static const Pose kPoseStep1 = {
  FrontLeftLeg{642, 325, 840}, FrontRightLeg{381, 808, 187},
  MiddleLeftLeg{512, 183, 839}, MiddleRightLeg{512, 701, 182},
  BackLeftLeg{382, 326, 839}, BackRightLeg{637, 813, 181},
};

// left middle, right front and back up and fwd, other down and bwd
static const Pose kPoseStep2 = {
  FrontLeftLeg{690, 326, 840}, FrontRightLeg{443, 808, 187},
  MiddleLeftLeg{454, 183, 839}, MiddleRightLeg{455, 701, 183},
  BackLeftLeg{432, 326, 839}, BackRightLeg{681, 813, 181},
};

// left middle, right front and back fwd, other bwd, all down
static const Pose kPoseStep3 = {
  FrontLeftLeg{690, 325, 841}, FrontRightLeg{442, 662, 258},
  MiddleLeftLeg{455, 337, 786}, MiddleRightLeg{455, 701, 183},
  BackLeftLeg{432, 326, 839}, BackRightLeg{681, 677, 235},
};

// left middle, right front and back fwd and down, other bwd and up
static const Pose kPoseStep4 = {
  FrontLeftLeg{691, 202, 841}, FrontRightLeg{442, 662, 258},
  MiddleLeftLeg{455, 337, 786}, MiddleRightLeg{455, 826, 183},
  BackLeftLeg{431, 211, 839}, BackRightLeg{681, 677, 235},
};

// left middle, right front and back bwd and down, other fwd and up
static const Pose kPoseStep5 = {
  FrontLeftLeg{566, 202, 842}, FrontRightLeg{346, 662, 258},
  MiddleLeftLeg{561, 337, 786}, MiddleRightLeg{588, 826, 183},
  BackLeftLeg{338, 212, 840}, BackRightLeg{578, 677, 235},
};

// left middle, right front and back bwd, other fwd, all down
static const Pose kPoseStep6 = {
  FrontLeftLeg{566, 370, 810}, FrontRightLeg{346, 637, 283},
  MiddleLeftLeg{560, 337, 786}, MiddleRightLeg{588, 645, 216},
  BackLeftLeg{338, 350, 766}, BackRightLeg{580, 627, 234},
};

// left middle, right front and back bwd and up, other fwd and up
static const Pose kPoseStep7 = {
  FrontLeftLeg{567, 370, 810}, FrontRightLeg{346, 792, 208},
  MiddleLeftLeg{561, 207, 829}, MiddleRightLeg{588, 645, 216},
  BackLeftLeg{337, 350, 766}, BackRightLeg{579, 810, 177},
};

static const Pose kWalkSeq[] = {
  kStandingNarrowPose,
  kPoseStep1,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep2,
  kPoseStep3,
  kPoseStep4,
  kPoseStep5,
  kPoseStep6,
  kPoseStep7,
  kPoseStep1,
  kStandingNarrowPose
};
// clang-format on
