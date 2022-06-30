#pragma once

#include <array>
#include <inttypes.h>
#include <numeric>
#include <string>
#include <ranges>

using ServoId = uint8_t;
enum LegId {
  // Head is at 12 o'clock

  kFrontRightLegId = 0, //  1 o'clock
  kFrontLeftLegId,      // 11 o'clock

  kMiddleRightLegId, // 3 o'clock
  kMiddleLeftLegId,  // 9 o'clock

  kBackRightLegId, // 5 o'clock
  kBackLeftLegId,  // 7 o'clock

  // Must be last
  kLegIdCount
};

constexpr size_t kServosPerLeg = 3;
constexpr const ServoId kServoMinId = 1;
constexpr const ServoId kServoMaxId = kServoMinId + kServosPerLeg * kLegIdCount;

using ServoIdsArray = std::array<ServoId, kServoMaxId - kServoMinId>;

ServoIdsArray servoIdsRange() {
  ServoIdsArray result;
  std::iota(std::begin(result), std::end(result), kServoMinId);
  return result;
}

using LegServoIdsArray = std::array<int, kServosPerLeg>;
using AllLegsServoIdsArray = std::array<LegServoIdsArray, kLegIdCount>;

const AllLegsServoIdsArray kAllLegServoIds = []() {
  AllLegsServoIdsArray legs;
  legs[kFrontRightLegId] = {2, 4, 6};
  legs[kFrontLeftLegId] = {1, 3, 5};

  legs[kMiddleRightLegId] = {14, 16, 18};
  legs[kMiddleLeftLegId] = {13, 15, 17};

  legs[kBackRightLegId] = {8, 10, 12};
  legs[kBackLeftLegId] = {7, 9, 11};

  return legs;
}();

using ServoIdToLegArray = std::array<ServoId, kServoMaxId>;

const ServoIdToLegArray kServoIdToLeg = []() {
  ServoIdToLegArray mappings;

  for (int leg = kFrontRightLegId; leg < kLegIdCount; ++leg) {
    for (const ServoId id : kAllLegServoIds[leg]) {
      mappings[id] = leg;
    }
  }

  return mappings;
}();

using AllLegsNamesArray = std::array<std::string, kLegIdCount>;
const AllLegsNamesArray kAllLegsNames = []() {
  AllLegsNamesArray legs;
  legs[kFrontRightLegId] = "front-right";
  legs[kFrontLeftLegId] = "front-left";

  legs[kMiddleRightLegId] = "middle-right";
  legs[kMiddleLeftLegId] = "middle-left";

  legs[kBackRightLegId] = "back-right";
  legs[kBackLeftLegId] = "back-left";

  return legs;
}();

std::string legNameForServo(ServoId id)
{
  return kAllLegsNames[kServoIdToLeg[id]];
}