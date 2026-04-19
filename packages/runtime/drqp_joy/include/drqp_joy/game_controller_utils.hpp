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

#ifndef DRQP_JOY__GAME_CONTROLLER_UTILS_HPP_
#define DRQP_JOY__GAME_CONTROLLER_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drqp_joy::detail
{

inline constexpr float kAxisChangeTolerance = 1.0e-6F;
inline constexpr int kResponsiveEventPollIntervalMs = 5;

inline void validateDeadzone(double deadzone)
{
  if (deadzone < 0.0 || deadzone >= 1.0) {
    throw std::runtime_error("Deadzone must be in the range [0.0, 1.0)");
  }
}

inline void validateCoalesceInterval(int interval_ms)
{
  if (interval_ms < 0) {
    throw std::runtime_error("coalesce_interval_ms must be non-negative");
  }
}

inline bool axisValueChanged(float previous_value, float next_value)
{
  return std::fabs(previous_value - next_value) > kAxisChangeTolerance;
}

inline int computeEventPollIntervalMs(int autorepeat_interval_ms, int coalesce_interval_ms)
{
  validateCoalesceInterval(coalesce_interval_ms);

  int interval_ms = kResponsiveEventPollIntervalMs;
  if (autorepeat_interval_ms > 0) {
    interval_ms = std::min(interval_ms, autorepeat_interval_ms);
  }
  if (coalesce_interval_ms > 0) {
    interval_ms = std::min(interval_ms, coalesce_interval_ms);
  }

  return std::max(1, interval_ms);
}

}  // namespace drqp_joy::detail

#endif  // DRQP_JOY__GAME_CONTROLLER_UTILS_HPP_
