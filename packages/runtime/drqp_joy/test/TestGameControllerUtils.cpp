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

#include <catch_amalgamated.hpp>
#include <catch_ros2/catch.hpp>

#include "drqp_joy/game_controller_utils.hpp"

namespace
{

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
  CHECK_EQ(drqp_joy::detail::computeEventPollIntervalMs(50, 0), 5);
  CHECK_EQ(drqp_joy::detail::computeEventPollIntervalMs(50, 1), 1);
  CHECK_EQ(drqp_joy::detail::computeEventPollIntervalMs(2, 10), 2);
}

TEST_CASE("computeEventPollIntervalMs never returns less than one millisecond")
{
  CHECK_EQ(
    drqp_joy::detail::computeEventPollIntervalMs(0, 0),
    drqp_joy::detail::kResponsiveEventPollIntervalMs);
  CHECK_EQ(drqp_joy::detail::computeEventPollIntervalMs(1, 0), 1);
}

}  // namespace
