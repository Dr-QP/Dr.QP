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

#ifndef DRQP_JOY__DUALSENSE_DIAGNOSTIC_UTILS_HPP_
#define DRQP_JOY__DUALSENSE_DIAGNOSTIC_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace drqp_joy::diagnostics::detail
{

template <typename IdType>
struct NamedDevice
{
  IdType id;
  std::string name;
};

inline std::string toLowerAscii(std::string_view text)
{
  std::string lowered;
  lowered.reserve(text.size());

  std::transform(text.begin(), text.end(), std::back_inserter(lowered), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  return lowered;
}

inline bool containsCaseInsensitive(std::string_view haystack, std::string_view needle)
{
  if (needle.empty()) {
    return true;
  }

  return toLowerAscii(haystack).find(toLowerAscii(needle)) != std::string::npos;
}

inline bool isLikelyDualSenseGamepadName(std::string_view name)
{
  return containsCaseInsensitive(name, "dualsense") ||
         containsCaseInsensitive(name, "wireless controller");
}

inline bool isLikelyDualSenseAudioDeviceName(std::string_view name)
{
  return containsCaseInsensitive(name, "dualsense") ||
         containsCaseInsensitive(name, "wireless controller") ||
         containsCaseInsensitive(name, "sony");
}

template <typename IdType>
inline auto findDeviceBySubstring(
  const std::vector<NamedDevice<IdType>>& devices, std::string_view name_substring)
  -> std::optional<NamedDevice<IdType>>
{
  const auto match = std::find_if(
    devices.begin(), devices.end(), [name_substring](const NamedDevice<IdType>& device) {
      return containsCaseInsensitive(device.name, name_substring);
    });

  if (match == devices.end()) {
    return std::nullopt;
  }

  return *match;
}

template <typename IdType, typename Predicate>
inline auto findFirstMatchingDevice(
  const std::vector<NamedDevice<IdType>>& devices, Predicate predicate)
  -> std::optional<NamedDevice<IdType>>
{
  const auto match = std::find_if(
    devices.begin(), devices.end(),
    [&predicate](const NamedDevice<IdType>& device) { return predicate(device.name); });

  if (match == devices.end()) {
    return std::nullopt;
  }

  return *match;
}

}  // namespace drqp_joy::diagnostics::detail

#endif  // DRQP_JOY__DUALSENSE_DIAGNOSTIC_UTILS_HPP_
