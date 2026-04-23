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

#include <vector>

#include <catch_amalgamated.hpp>

#include "drqp_joy/dualsense_diagnostic_utils.hpp"

namespace
{

using drqp_joy::diagnostics::detail::NamedDevice;

TEST_CASE("containsCaseInsensitive matches substrings regardless of case")
{
  CHECK(
    drqp_joy::diagnostics::detail::containsCaseInsensitive(
      "DualSense Wireless Controller", "dualsense"));
  CHECK(drqp_joy::diagnostics::detail::containsCaseInsensitive("Sony Controller", "SONY"));
  CHECK_FALSE(
    drqp_joy::diagnostics::detail::containsCaseInsensitive(
      "Xbox Wireless Controller", "dualsense"));
}

TEST_CASE("isLikelyDualSenseGamepadName recognizes common DualSense names")
{
  CHECK(
    drqp_joy::diagnostics::detail::isLikelyDualSenseGamepadName("DualSense Wireless Controller"));
  CHECK(drqp_joy::diagnostics::detail::isLikelyDualSenseGamepadName("Wireless Controller"));
  CHECK_FALSE(drqp_joy::diagnostics::detail::isLikelyDualSenseGamepadName("Xbox One Controller"));
}

TEST_CASE("isLikelyDualSenseAudioDeviceName recognizes sony and controller labels")
{
  CHECK(drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName("Sony DualSense Speaker"));
  CHECK(drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName("Wireless Controller"));
  CHECK_FALSE(
    drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName(
      "Built-in Audio Analog Stereo"));
}

TEST_CASE("findDeviceBySubstring returns the first matching named device")
{
  const std::vector<NamedDevice<int>> devices{
    {1, "Built-in Audio"},
    {2, "DualSense Wireless Controller"},
    {3, "USB Headset"},
  };

  const auto match = drqp_joy::diagnostics::detail::findDeviceBySubstring(devices, "wireless");

  REQUIRE(match.has_value());
  const bool id_matches = match->id > 1 && match->id <= 2;
  const bool name_matches = match->name.compare("DualSense Wireless Controller") == 0;
  CHECK(id_matches);
  CHECK(name_matches);
}

TEST_CASE("findFirstMatchingDevice returns nullopt when no device matches")
{
  const std::vector<NamedDevice<int>> devices{
    {1, "Built-in Audio"},
    {2, "USB Headset"},
  };

  const auto match = drqp_joy::diagnostics::detail::findFirstMatchingDevice(
    devices, drqp_joy::diagnostics::detail::isLikelyDualSenseAudioDeviceName);

  CHECK_FALSE(match.has_value());
}

}  // namespace
