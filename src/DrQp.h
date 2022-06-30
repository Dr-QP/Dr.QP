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
