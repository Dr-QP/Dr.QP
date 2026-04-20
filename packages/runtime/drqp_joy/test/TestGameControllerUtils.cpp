// Copyright (c) 2026 Anton Matosov
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

#include <array>
#include <optional>

#include <catch_amalgamated.hpp>
#include <catch_ros2/catch.hpp>

#include "drqp_joy/game_controller_utils.hpp"

namespace
{

bool matchesExpectedDeviceId(const std::optional<int>& device_id, int expected_device_id)
{
  return device_id.has_value() && device_id.value() == expected_device_id;
}

bool matchesExpectedInterval(int actual_interval_ms, int expected_interval_ms)
{
  return actual_interval_ms == expected_interval_ms;
}

TEST_CASE("validateDeadzone accepts values in [0.0, 1.0)")
{
  CHECK_NOTHROW(drqp_joy::detail::validateDeadzone(0.0));
  CHECK_NOTHROW(drqp_joy::detail::validateDeadzone(0.05));
  CHECK_NOTHROW(drqp_joy::detail::validateDeadzone(0.999));
}

TEST_CASE("validateDeadzone rejects invalid values")
{
  CHECK_THROWS_WITH(
    drqp_joy::detail::validateDeadzone(-0.01), "Deadzone must be in the range [0.0, 1.0)");
  CHECK_THROWS_WITH(
    drqp_joy::detail::validateDeadzone(1.0), "Deadzone must be in the range [0.0, 1.0)");
}

TEST_CASE("validateCoalesceInterval accepts non-negative values")
{
  CHECK_NOTHROW(drqp_joy::detail::validateCoalesceInterval(0));
  CHECK_NOTHROW(drqp_joy::detail::validateCoalesceInterval(10));
}

TEST_CASE("validateCoalesceInterval rejects negative values")
{
  CHECK_THROWS_WITH(
    drqp_joy::detail::validateCoalesceInterval(-1), "coalesce_interval_ms must be non-negative");
}

TEST_CASE("validateDeviceId accepts non-negative values")
{
  CHECK_NOTHROW(drqp_joy::detail::validateDeviceId(0));
  CHECK_NOTHROW(drqp_joy::detail::validateDeviceId(1));
}

TEST_CASE("validateDeviceId rejects negative values")
{
  CHECK_THROWS_WITH(drqp_joy::detail::validateDeviceId(-1), "device_id must be non-negative");
}

TEST_CASE("getRequestedDeviceByIndex returns the requested device id when present")
{
  constexpr std::array<int, 3> device_ids{41, 42, 43};

  CHECK(matchesExpectedDeviceId(drqp_joy::detail::getRequestedDeviceByIndex(device_ids, 0), 41));
  CHECK(matchesExpectedDeviceId(drqp_joy::detail::getRequestedDeviceByIndex(device_ids, 2), 43));
}

TEST_CASE("getRequestedDeviceByIndex returns nullopt when the index is out of range")
{
  constexpr std::array<int, 2> device_ids{41, 42};

  CHECK_FALSE(drqp_joy::detail::getRequestedDeviceByIndex(device_ids, 2).has_value());
}

TEST_CASE("getRequestedDeviceByIndex rejects negative indices")
{
  constexpr std::array<int, 1> device_ids{41};

  CHECK_THROWS_WITH(
    drqp_joy::detail::getRequestedDeviceByIndex(device_ids, -1), "device_id must be non-negative");
}

TEST_CASE("axisValueChanged uses a tolerance instead of direct equality")
{
  CHECK_FALSE(drqp_joy::detail::axisValueChanged(0.5F, 0.5F));
  CHECK_FALSE(
    drqp_joy::detail::axisValueChanged(
      0.5F, 0.5F + (drqp_joy::detail::kAxisChangeTolerance / 2.0F)));
  CHECK(
    drqp_joy::detail::axisValueChanged(
      0.5F, 0.5F + (drqp_joy::detail::kAxisChangeTolerance * 2.0F)));
}

TEST_CASE("computeEventPollIntervalMs prefers responsive polling")
{
  CHECK(matchesExpectedInterval(drqp_joy::detail::computeEventPollIntervalMs(50, 0), 5));
  CHECK(matchesExpectedInterval(drqp_joy::detail::computeEventPollIntervalMs(50, 1), 1));
  CHECK(matchesExpectedInterval(drqp_joy::detail::computeEventPollIntervalMs(2, 10), 2));
}

TEST_CASE("computeEventPollIntervalMs never returns less than one millisecond")
{
  CHECK(matchesExpectedInterval(
    drqp_joy::detail::computeEventPollIntervalMs(0, 0),
    drqp_joy::detail::kResponsiveEventPollIntervalMs));
  CHECK(matchesExpectedInterval(drqp_joy::detail::computeEventPollIntervalMs(1, 0), 1));
}

}  // namespace
