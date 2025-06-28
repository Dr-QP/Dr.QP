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


fs::path RobotConfig::getConfigPath()
{
  fs::path yamlPath = node_->get_parameter("config").as_string();
  if (yamlPath.empty()) {
    const fs::path packageShareDir = ament_index_cpp::get_package_share_directory("drqp_control");
    yamlPath = packageShareDir / "config" / "drqp.yml";
  }
  if (!fs::exists(yamlPath)) {
    RCLCPP_ERROR(node_->get_logger(), "%s could not be found. Exiting.", yamlPath.c_str());
    throw std::runtime_error("Robot config parsing failure.");
  }
  return yamlPath;
}

void RobotConfig::loadConfig(fs::path configPath)
{
  try {
    if (configPath.empty()) {
      configPath = getConfigPath();
    }
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
      RCLCPP_INFO(node_->get_logger(), "Robot namespace: %s", robotNamespace.c_str());
    }

    // device_address
    if (YAML::Node deviceAddress = robot["device_address"]; deviceAddress) {
      node_->set_parameter(rclcpp::Parameter("device_address", deviceAddress.as<std::string>()));
    }
    // baud_rate
    if (YAML::Node baudRate = robot["baud_rate"]; baudRate) {
      node_->set_parameter(rclcpp::Parameter("baud_rate", baudRate.as<int>()));
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

      const double ratio = inverted ? -1. : 1.;

      jointToServoId_[name] = ServoParams{id, ratio, offset_rads};
      servoIdToJoint_[id] = JointParams{name, ratio, offset_rads};
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config %s", e.what());
    throw;
  }
}

std::optional<ServoValues> RobotConfig::jointToServo(const JointValues& joint)
{
  if (jointToServoId_.count(joint.name) == 0) {
    return std::nullopt;
  }

  const ServoParams servoParams = jointToServoId_.at(joint.name);
  const uint16_t position = radiansToPosition(joint.position_as_radians * servoParams.ratio + servoParams.offset_rads);
  return ServoValues{servoParams.id, position};
}

std::optional<JointValues> RobotConfig::servoToJoint(const ServoValues& servo)
{
  if (servoIdToJoint_.count(servo.id) == 0) {
    return std::nullopt;
  }

  const JointParams jointParams = servoIdToJoint_.at(servo.id);
  const double positionAsRadians = positionToRadians(servo.position) * jointParams.ratio - jointParams.offset_rads;
  return JointValues{jointParams.jointName, positionAsRadians};
}
