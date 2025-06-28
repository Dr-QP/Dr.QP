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
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "drqp_control/DrQp.h"

namespace fs = std::filesystem;
struct ServoParams
{
  uint8_t id;
  double ratio = 1.;
  double offset_rads = 0.;
};

struct JointParams
{
  std::string jointName;
  double ratio = 1.;
  double offset_rads = 0.;
};

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
class RobotConfig
{
public:
  RobotConfig(rclcpp::Node* node) : node_(node) {}

  fs::path getConfigPath();

  void loadConfig(fs::path configPath = {});

  std::optional<ServoValues> jointToServo(const JointValues& joint);

  std::optional<JointValues> servoToJoint(const ServoValues& servo);

private:
  rclcpp::Node* node_;
  std::unordered_map<std::string, ServoParams> jointToServoId_;
  std::unordered_map<uint8_t, JointParams> servoIdToJoint_;
};
