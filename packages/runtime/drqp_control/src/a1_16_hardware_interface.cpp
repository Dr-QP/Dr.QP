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
#include <rclcpp/logging.hpp>

#include <drqp_serial/SerialFactory.h>
#include <drqp_a1_16_driver/XYZrobotServo.h>

#include "drqp_control/DrQp.h"
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

hardware_interface::CallbackReturn a1_16_hardware_interface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (auto status = hardware_interface::SystemInterface::on_init(info);
      status != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize hardware interface.");
    return status;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // Dr.QP has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto& commandInterface = joint.command_interfaces[0];
    if (commandInterface.name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), commandInterface.name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

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

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  servoSerial_ = makeSerialForDevice(get_param(info.hardware_parameters, "device_address"));
  servoSerial_->begin(std::stoi(get_param(info.hardware_parameters, "baud_rate")));

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
  // reset values always when configuring hardware
  for (const auto& [name, descr] : joint_state_interfaces_) {
    set_state(name, 0.0);
  }
  for (const auto& [name, descr] : joint_command_interfaces_) {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type a1_16_hardware_interface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  // std::stringstream ss;
  // ss << "Reading states:";
  // auto hw_slowdown_ = 10.;
  // for (const auto & [name, descr] : joint_state_interfaces_)
  // {
  //   // Simulate RRBot's movement
  //   auto new_value = get_state(name) + (get_command(name) - get_state(name)) / hw_slowdown_;
  //   set_state(name, new_value);
  //   ss << std::fixed << std::setprecision(2) << std::endl
  //      << "\t" << get_state(name) << " for joint '" << name << "'";
  // }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type a1_16_hardware_interface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

  DynamicIJogCommand iposCmd(joint_command_interfaces_.size());
  size_t index = 0;
  for (const auto& [name, descr] : joint_command_interfaces_) {
    auto pos = get_command(name);

    auto servoValues = robotConfig_.jointToServo({descr.prefix_name, pos});
    if (!servoValues) {
      RCLCPP_ERROR(get_logger(), "Unknown joint name %s", descr.prefix_name.c_str());
      continue;
    }
    // if (!torqueIsOn_[servoValues->id]) {
    //   continue;
    // }
    iposCmd.at(index) = {servoValues->position, SET_POSITION_CONTROL, servoValues->id, 0};
    index++;

    // Convert back, this will handle invertion, clamping and offset
    auto jointState = robotConfig_.servoToJoint({servoValues->id, servoValues->position});
    set_state(name, jointState->position_as_radians);
  }
  servo.sendJogCommand(iposCmd);

  return hardware_interface::return_type::OK;
}
}  // namespace drqp_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(drqp_control::a1_16_hardware_interface, hardware_interface::SystemInterface)
