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

#include "drqp_control/RobotConfig.h"

#include <yaml-cpp/yaml.h>

#include "drqp_control/DrQp.h"

struct RobotConfig::ServoParams
{
  uint8_t id;
  double ratio = 1.;
  double offset_rads = 0.;
  double min_angle_rads = -M_PI;
  double max_angle_rads = M_PI;
};

struct RobotConfig::JointParams
{
  std::string joint_name;
  double ratio = 1.;
  double offset_rads = 0.;
  double min_angle_rads = -M_PI;
  double max_angle_rads = M_PI;
};

RobotConfig::RobotConfig() : logger_(rclcpp::get_logger("RobotConfig")) {}

RobotConfig::RobotConfig(rclcpp::Logger logger) : logger_(logger) {}

RobotConfig::~RobotConfig() = default;

void RobotConfig::loadConfig(fs::path configPath)
{
  try {
    YAML::Node config = YAML::LoadFile(configPath.string());
    if (!config) {
      throw std::runtime_error("Robot config parsing failure.");
    }
    YAML::Node robot = config["robot"];
    if (!robot) {
      throw std::runtime_error("Robot config parsing failure. No 'robot' section.");
    }

    // namespace
    std::string robotNamespace = "";
    if (YAML::Node namespaceNode = robot["namespace"]; namespaceNode) {
      robotNamespace = namespaceNode.as<std::string>() + "/";
    }

    // device_address
    if (YAML::Node deviceAddress = robot["device_address"]; deviceAddress) {
      deviceAddress_ = deviceAddress.as<std::string>();
    }

    // baud_rate
    if (YAML::Node baudRate = robot["baud_rate"]; baudRate) {
      baudRate_ = baudRate.as<int>();
    }

    // servos
    YAML::Node servos = robot["servos"];
    if (!servos) {
      throw std::runtime_error("Robot config parsing failure. No 'servos' section.");
    }
    for (const auto& servo : servos) {
      const std::string name = robotNamespace + servo.first.as<std::string>();
      const uint8_t id = servo.second["id"].as<uint8_t>();
      double offset_rads = 0.;
      if (servo.second["offset_rads"]) {
        offset_rads = servo.second["offset_rads"].as<double>();
      }
      bool inverted = false;
      if (servo.second["inverted"]) {
        inverted = servo.second["inverted"].as<bool>();
      }
      double min_rads = -M_PI;
      if (servo.second["min_rads"]) {
        min_rads = servo.second["min_rads"].as<double>();
      }
      double max_rads = M_PI;
      if (servo.second["max_rads"]) {
        max_rads = servo.second["max_rads"].as<double>();
      }

      addServo(
        ServoJointParams{
          .joint_name = name,
          .servo_id = id,
          .inverted = inverted,
          .offset_radians = offset_rads,
          .min_angle_radians = min_rads,
          .max_angle_radians = max_rads,
        });
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load config %s", e.what());
    throw;
  }
}

void RobotConfig::addServo(const ServoJointParams& params)
{
  const double ratio = params.inverted ? -1. : 1.;

  const auto true_min_rads = std::min(params.min_angle_radians, params.max_angle_radians);
  const auto true_max_rads = std::max(params.min_angle_radians, params.max_angle_radians);

  jointToServoId_[params.joint_name] = ServoParams{
    .id = params.servo_id,
    .ratio = ratio,
    .offset_rads = params.offset_radians,
    .min_angle_rads = true_min_rads,
    .max_angle_rads = true_max_rads};
  servoIdToJoint_[params.servo_id] = JointParams{
    .joint_name = params.joint_name,
    .ratio = ratio,
    .offset_rads = params.offset_radians,
    .min_angle_rads = true_min_rads,
    .max_angle_rads = true_max_rads};
}

double safeClamp(double value, double min, double max)
{
  if (std::isnan(value)) {
    return min;
  }
  if (std::isinf(value)) {
    return value > 0 ? max : min;
  }
  return std::clamp(value, min, max);
}

std::optional<RobotConfig::ServoValues> RobotConfig::jointToServo(const JointValues& joint)
{
  if (jointToServoId_.count(joint.name) == 0) {
    return std::nullopt;
  }

  const ServoParams servoParams = jointToServoId_.at(joint.name);
  const auto rawPosition = joint.position_as_radians + servoParams.offset_rads;
  const double clampedPosition =
    safeClamp(rawPosition, servoParams.min_angle_rads, servoParams.max_angle_rads);
  const uint16_t position = radiansToPosition(clampedPosition * servoParams.ratio);
  return ServoValues{.id = servoParams.id, .position = position};
}

std::optional<RobotConfig::JointValues> RobotConfig::servoToJoint(const ServoValues& servo)
{
  if (servoIdToJoint_.count(servo.id) == 0) {
    return std::nullopt;
  }

  const JointParams jointParams = servoIdToJoint_.at(servo.id);
  const double positionAsRadians =
    positionToRadians(servo.position) * jointParams.ratio - jointParams.offset_rads;
  return JointValues{.name = jointParams.joint_name, .position_as_radians = positionAsRadians};
}

std::vector<std::string> RobotConfig::getJointNames() const
{
  std::vector<std::string> jointNames;
  for (const auto& [name, _] : jointToServoId_) {
    jointNames.push_back(name);
  }
  return jointNames;
}

std::vector<uint8_t> RobotConfig::getServoIds() const
{
  std::vector<uint8_t> servoIds;
  for (const auto& [id, _] : servoIdToJoint_) {
    servoIds.push_back(id);
  }
  return servoIds;
}
////////////////////////////////////////////////////////////////////////

NodeRobotConfig::NodeRobotConfig(rclcpp::Node* node) : RobotConfig(node->get_logger()), node_(node)
{
  node_->declare_parameter("config", "");
}

fs::path NodeRobotConfig::getConfigPath()
{
  fs::path yamlPath = node_->get_parameter("config").as_string();
  if (yamlPath.empty()) {
    const fs::path packageShareDir = ament_index_cpp::get_package_share_directory("drqp_control");
    yamlPath = packageShareDir / "config" / "drqp.yml";
  }
  if (!fs::exists(yamlPath)) {
    RCLCPP_ERROR(get_logger(), "%s could not be found. Exiting.", yamlPath.c_str());
    throw std::runtime_error("Robot config parsing failure.");
  }
  return yamlPath;
}

void NodeRobotConfig::loadConfig()
{
  loadConfig(getConfigPath());
}

void NodeRobotConfig::loadConfig(fs::path configPath)
{
  RobotConfig::loadConfig(configPath);
  declareParameters();
}

void NodeRobotConfig::declareParameters()
{
  node_->declare_parameter("device_address", deviceAddress_);
  node_->declare_parameter("baud_rate", baudRate_);
}
