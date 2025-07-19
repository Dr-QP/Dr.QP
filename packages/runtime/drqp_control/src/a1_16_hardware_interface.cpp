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

a1_16_hardware_interface::a1_16_hardware_interface() = default;
a1_16_hardware_interface::~a1_16_hardware_interface() = default;

hardware_interface::CallbackReturn a1_16_hardware_interface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (auto status = hardware_interface::SystemInterface::on_init(info);
      status != hardware_interface::CallbackReturn::SUCCESS) {
    return status;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

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

  return hardware_interface::CallbackReturn::SUCCESS;
}

// std::vector<hardware_interface::StateInterface> a1_16_hardware_interface::export_state_interfaces()
// {
//   return {};
// }

// std::vector<hardware_interface::CommandInterface> a1_16_hardware_interface::export_command_interfaces()
// {
//   return {};
// }

hardware_interface::CallbackReturn a1_16_hardware_interface::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn a1_16_hardware_interface::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type a1_16_hardware_interface::read(
  const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type a1_16_hardware_interface::write(
  const rclcpp::Time& time, const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}
