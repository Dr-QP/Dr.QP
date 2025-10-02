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

#include "drqp_control/a1_16_hardware_interface.h"
#include <cassert>
#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>

#include "drqp_serial/SerialFactory.h"
#include "drqp_a1_16_driver/XYZrobotServo.h"
#include "drqp_a1_16_driver/MockServo.h"

namespace drqp_control
{
a1_16_hardware_interface::a1_16_hardware_interface() {}
a1_16_hardware_interface::~a1_16_hardware_interface() = default;

std::string get_param(
  const std::unordered_map<std::string, std::string>& parameters, const std::string& key)
{
  if (parameters.count(key) == 0) {
    throw std::runtime_error("Missing parameter " + key);
  }
  return parameters.at(key);
}

bool get_bool_param(
  const std::unordered_map<std::string, std::string>& parameters, const std::string& key)
{
  auto value = get_param(parameters, key);
  if (value != "True" && value != "False") {
    throw std::runtime_error(
      "Invalid boolean value '" + value + "' for parameter '" + key +
      "'. Expected 'True' or 'False'.");
  }
  return value == "True";
}

a1_16_hardware_interface::ServoPtr a1_16_hardware_interface::makeServo(uint8_t id)
{
  if (useMockServo_) {
    return std::make_unique<MockServo>(id);
  }
  return std::make_unique<XYZrobotServo>(*servoSerial_, id);
}

hardware_interface::CallbackReturn a1_16_hardware_interface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (auto status = hardware_interface::SystemInterface::on_init(info);
      status != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize hardware interface.");
    return status;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    for (const auto& commandInterface : joint.command_interfaces) {
      if (commandInterface.name == hardware_interface::HW_IF_POSITION) {
        uint8_t servoId = std::stoi(get_param(commandInterface.parameters, "servo_id"));
        robotConfig_.addServo(
          RobotConfig::ServoJointParams{
            .joint_name = joint.name,
            .servo_id = servoId,
            .inverted = get_bool_param(commandInterface.parameters, "inverted"),
            .offset_radians = std::stod(get_param(commandInterface.parameters, "offset_rads")),
            .min_angle_radians = std::stod(get_param(commandInterface.parameters, "min")),
            .max_angle_radians = std::stod(get_param(commandInterface.parameters, "max")),
          });
      }
    }
  }

  auto deviceAddress = get_param(info.hardware_parameters, "device_address");
  RCLCPP_INFO(get_logger(), "Connecting to %s", deviceAddress.c_str());
  if (deviceAddress == "mock_servo") {
    useMockServo_ = true;
  } else {
    useMockServo_ = false;
    servoSerial_ = makeSerialForDevice(deviceAddress);
    servoSerial_->begin(std::stoi(get_param(info.hardware_parameters, "baud_rate")));
  }

  for (const hardware_interface::ComponentInfo& sensor : info_.sensors) {
    for (const auto& stateInterface : sensor.state_interfaces) {
      if (stateInterface.name == "voltage") {
        batteryServoId_ = std::stoi(get_param(stateInterface.parameters, "servo_id"));
        if (useMockServo_) {
          set_state("battery_state/voltage", std::stod(get_param(stateInterface.parameters, "initial_value")));
        }
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn a1_16_hardware_interface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn a1_16_hardware_interface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn a1_16_hardware_interface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Reset all interfaces to 0.0
  for (const auto& [name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto& [name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }

  try {
    // Read all servos and set commands to current position
    for (const auto servoId : robotConfig_.getServoIds()) {
      readServoStatus(servoId);
    }
    for (const auto& jointName : robotConfig_.getJointNames()) {
      const auto position = get_state(jointName + "/position");
      set_command(jointName + "/position", position);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to configure hardware interface: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type a1_16_hardware_interface::readServoStatus(uint8_t servoId)
{
  try {
    ServoPtr servo = makeServo(servoId);

    if (servoId == batteryServoId_) {
      uint8_t voltage = 0;
      servo->ramRead(offsetof(XYZrobotServoRAM, Voltage), &voltage, sizeof(voltage));
      if (servo->isFailed()) {
        RCLCPP_ERROR(
          get_logger(), "Failed to read RAM for servo %i: %s", servoId,
          to_string(servo->getLastError()).c_str());
        return hardware_interface::return_type::OK;
      }
      set_state("battery_state/voltage", voltage / 16.0);
    }

    XYZrobotServoStatus status = servo->readStatus();
    if (servo->isFailed()) {
      RCLCPP_ERROR(
        get_logger(), "Failed to read status for servo %i: %s", servoId,
        to_string(servo->getLastError()).c_str());
      return hardware_interface::return_type::OK;
    }

    auto jointValues = robotConfig_.servoToJoint({servoId, status.position});
    if (!jointValues) {
      RCLCPP_ERROR(get_logger(), "Failed to convert servo %i position to joint", servoId);
      return hardware_interface::return_type::OK;
    }
    set_state(jointValues->name + "/position", jointValues->position_as_radians);
    set_state(jointValues->name + "/pwm", status.pwm / 1023.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to read servo %i: %s", servoId, e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to read servo %i: unknown exception", servoId);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type a1_16_hardware_interface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // Read one servo at a time, round robin
  try {
    const auto servoIds = robotConfig_.getServoIds();
    const uint8_t servoId = servoIds[servoIndexLastRead_];

    if (torqueIsOn_[servoId] == ServoTorque::On) {
      return hardware_interface::return_type::OK;
    }
    servoIndexLastRead_ = (servoIndexLastRead_ + 1) % servoIds.size();

    return readServoStatus(servoId);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to read: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to read: unknown exception");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type a1_16_hardware_interface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  ServoPtr servo = makeServo(XYZrobotServo::kBroadcastId);

  std::vector<IJogData> iposCmd;
  iposCmd.reserve(robotConfig_.numServos());
  for (const auto& jointName : robotConfig_.getJointNames()) {
    auto effort = get_command(jointName + "/effort");
    effort = safeClamp(effort, -1.0, 1.0);

    auto pos = get_command(jointName + "/position");

    auto servoValues = robotConfig_.jointToServo({jointName, pos});
    assert(servoValues);
    uint8_t servoCommand = SET_POSITION_CONTROL;
    if (effort < 0.0) {
      if (torqueIsOn_[servoValues->id] == ServoTorque::Reboot) {
        // Servo is already rebooting, no need to do anything
        continue;
      }
      torqueIsOn_[servoValues->id] = ServoTorque::Reboot;
      ServoPtr servo = makeServo(servoValues->id);
      servo->reboot();
      continue;
    } else if (effort < 0.1) {
      if (torqueIsOn_[servoValues->id] == ServoTorque::Off) {
        // Servo is already off, no need to do anything
        continue;
      }
      torqueIsOn_[servoValues->id] = ServoTorque::Off;
      servoCommand = SET_TORQUE_OFF;
    } else if (torqueIsOn_[servoValues->id] != ServoTorque::On) {
      torqueIsOn_[servoValues->id] = ServoTorque::On;
      servoCommand = SET_POSITION_CONTROL_SERVO_ON;
    }

    iposCmd.emplace_back(
      IJogData{
        servoValues->position, servoCommand, servoValues->id,
        toPlaytime(period.to_chrono<std::chrono::milliseconds>())});

    if (torqueIsOn_[servoValues->id] == ServoTorque::On) {
      // Convert back, this will handle invertion, clamping and offset
      auto jointState = robotConfig_.servoToJoint({servoValues->id, servoValues->position});
      set_state(jointName + "/position", jointState->position_as_radians);
    }
  }
  servo->sendJogCommand(iposCmd);

  return hardware_interface::return_type::OK;
}
}  // namespace drqp_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(drqp_control::a1_16_hardware_interface, hardware_interface::SystemInterface)
