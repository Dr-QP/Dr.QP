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

#include <drqp_serial/SerialProtocol.h>
#include <drqp_serial/TcpSerial.h>
#include <drqp_serial/SerialFactory.h>
#include <drqp_a1_16_driver/XYZrobotServo.h>

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <filesystem>
namespace fs = std::filesystem;

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>

#include <drqp_interfaces/msg/multi_servo_state.hpp>
#include "drqp_control/JointServoMappings.h"

using namespace std::chrono_literals;

class PoseReader : public rclcpp::Node
{
public:
  PoseReader() : Node("drqp_pose_reader")
  {
    declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("baud_rate", 115200);
    declare_parameter("first_id", 1);
    declare_parameter("last_id", 18);
    declare_parameter("period_ms", 100);
    declare_parameter("config", "");

    const auto configPath = getConfigPath();
    loadConfig(configPath);

    servoStatesPublisher_ =
      this->create_publisher<drqp_interfaces::msg::MultiServoState>("/servo_states", 10);

    servoSerial_ = makeSerialForDevice(get_parameter("device_address").as_string());
    servoSerial_->begin(get_parameter("baud_rate").as_int());
    const uint8_t firstId = get_parameter("first_id").as_int();
    const uint8_t lastId = get_parameter("last_id").as_int();

    auto timerPeriod = std::chrono::milliseconds(get_parameter("period_ms").as_int());
    timer_ = this->create_wall_timer(timerPeriod, [this, firstId, lastId]() {
      try {
        auto multiServoStates = drqp_interfaces::msg::MultiServoState{};
        multiServoStates.header.stamp = this->get_clock()->now();

        for (uint8_t servoId = firstId; servoId <= lastId; ++servoId) {
          auto startTime = this->get_clock()->now();
          XYZrobotServo servo(*servoSerial_, servoId);

          XYZrobotServoStatus status = servo.readStatus();
          if (servo.isFailed()) {
            RCLCPP_ERROR(
              get_logger(), "Servo %i read status failed %s.", servoId,
              to_string(servo.getLastError()).c_str());
            continue;
          }
          auto servoState = drqp_interfaces::msg::ServoState{};
          auto endTime = this->get_clock()->now();

          servoState.read_duration_microsec = (endTime - startTime).nanoseconds() / 1000;
          servoState.raw.id = servoId;
          servoState.raw.position = status.position;
          servoState.raw.goal = status.posRef;
          servoState.raw.status_error = static_cast<uint8_t>(status.statusError);
          servoState.raw.status_detail = static_cast<uint8_t>(status.statusDetail);
          servoState.raw.torque = status.pwm;

          if (std::optional<JointValues> joint = servoToJoint({servoId, status.position})) {
            servoState.joint_name = joint->name;
            servoState.position_as_radians = joint->position_as_radians;
          }

          multiServoStates.servos.emplace_back(std::move(servoState));
        }
        auto endTime = this->get_clock()->now();
        multiServoStates.read_duration_microsec =
          (endTime - multiServoStates.header.stamp).nanoseconds() / 1000;
        servoStatesPublisher_->publish(multiServoStates);
      } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception occurred in pose read handler %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose read handler.");
      }
    });
  }

  fs::path getConfigPath()
  {
    fs::path yamlPath = get_parameter("config").as_string();
    if (yamlPath.empty()) {
      const fs::path packageShareDir = ament_index_cpp::get_package_share_directory("drqp_control");
      yamlPath = packageShareDir / "config" / "drqp.yml";
    }
    if (!fs::exists(yamlPath)) {
      RCLCPP_ERROR(this->get_logger(), "%s could not be found. Exiting.", yamlPath.c_str());
      throw std::runtime_error("Robot config parsing failure.");
    }
    return yamlPath;
  }

  void loadConfig(const fs::path& configPath)
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
        RCLCPP_INFO(this->get_logger(), "Robot namespace: %s", robotNamespace.c_str());
      }

      // device_address
      if (YAML::Node deviceAddress = robot["device_address"]; deviceAddress) {
        set_parameter(rclcpp::Parameter("device_address", deviceAddress.as<std::string>()));
      }
      // baud_rate
      if (YAML::Node baudRate = robot["baud_rate"]; baudRate) {
        set_parameter(rclcpp::Parameter("baud_rate", baudRate.as<int>()));
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
      RCLCPP_ERROR(this->get_logger(), "Failed to load config %s", e.what());
      throw;
    }
  }

  std::optional<ServoValues> jointToServo(const JointValues& joint)
  {
    if (jointToServoId_.count(joint.name) == 0) {
      return std::nullopt;
    }

    const ServoParams servoParams = jointToServoId_.at(joint.name);
    const uint16_t position = radiansToPosition(joint.position_as_radians * servoParams.ratio);
    return ServoValues{servoParams.id, position};
  }

  std::optional<JointValues> servoToJoint(const ServoValues& servo)
  {
    if (servoIdToJoint_.count(servo.id) == 0) {
      return std::nullopt;
    }

    const JointParams jointParams = servoIdToJoint_.at(servo.id);
    const double positionAsRadians = positionToRadians(servo.position) * jointParams.ratio;
    return JointValues{jointParams.jointName, positionAsRadians};
  }

  std::unordered_map<std::string, ServoParams> jointToServoId_;
  std::unordered_map<uint8_t, JointParams> servoIdToJoint_;

  std::unique_ptr<SerialProtocol> servoSerial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<drqp_interfaces::msg::MultiServoState>::SharedPtr servoStatesPublisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseReader>());
  rclcpp::shutdown();
  return 0;
}
