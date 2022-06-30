#pragma once

#include <array>
#include <inttypes.h>
#include <numeric>
#include <ranges>

constexpr const uint8_t kServoMinId = 1;
constexpr const uint8_t kServoMaxId = 18;

using ServoIdsArray = std::array<int, kServoMaxId - kServoMinId>;

ServoIdsArray servoIdsRange() {
  ServoIdsArray result;
  std::iota(std::begin(result), std::end(result), kServoMinId);
  return result;
}

enum LegId
{
  // Head is at 12 o'clock

  kFrontRightLegId = 0, //  1 o'clock
  kFrontLeftLegId, // 11 o'clock

  kMiddleRightLegId, // 3 o'clock
  kMiddleLeftLegId, // 9 o'clock

  kBackRightLegId, // 5 o'clock
  kBackLeftLegId, // 7 o'clock

  // Must be last
  kLegIdCount
};
using LegServoIdsArray = std::array<int, 3>;
using AllLegsServoIdsArray = std::array<LegServoIdsArray, kLegIdCount>;

const AllLegsServoIdsArray kAllLegs = [](){
  AllLegsServoIdsArray legs;
  legs[kFrontRightLegId] = {2, 4, 6};
  legs[kFrontLeftLegId] = {1, 3, 5};

  legs[kMiddleRightLegId] = {14, 16, 18};
  legs[kMiddleLeftLegId] = {13, 15, 17};

  legs[kBackRightLegId] = {8, 10, 12};
  legs[kBackLeftLegId] = {7, 9, 11};

  return legs;
}();