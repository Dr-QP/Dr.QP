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
a1_16_hardware_interface::a1_16_hardware_interface()
{

}
a1_16_hardware_interface::~a1_16_hardware_interface() = default;

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

    RCLCPP_DEBUG(get_logger(), "Joint %s", joint.name.c_str());
    RCLCPP_DEBUG(get_logger(), "  Command interfaces:");
    for (const auto& commandInterface : joint.command_interfaces) {
      RCLCPP_DEBUG(get_logger(), "    %s", commandInterface.name.c_str());
      RCLCPP_DEBUG(get_logger(), "      Command interface parameters:");
      for (const auto& [name, value] : commandInterface.parameters) {
        RCLCPP_DEBUG(get_logger(), "      %s = %s", name.c_str(), value.c_str());
      }

      robotConfig_.addServo(joint.name, std::stoi(commandInterface.parameters.at("id")),
        commandInterface.parameters.at("inverted") == "true",
        std::stod(commandInterface.parameters.at("offset_rads")));
    }
    RCLCPP_DEBUG(get_logger(), "  State interfaces:");
    for (const auto& stateInterface : joint.state_interfaces) {
      RCLCPP_DEBUG(get_logger(), "    %s", stateInterface.name.c_str());
      RCLCPP_DEBUG(get_logger(), "      State interface parameters:");
      for (const auto& [name, value] : stateInterface.parameters) {
        RCLCPP_DEBUG(get_logger(), "      %s = %s", name.c_str(), value.c_str());
      }
    }
    RCLCPP_DEBUG(get_logger(), "  Parameters:");
    for (const auto& [name, value] : joint.parameters) {
      RCLCPP_DEBUG(get_logger(), "    %s = %s", name.c_str(), value.c_str());
    }


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

  servoSerial_ = makeSerialForDevice(info.hardware_parameters.at("device_address"));
  servoSerial_->begin(std::stoi(info.hardware_parameters.at("baud_rate")));

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
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type a1_16_hardware_interface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_command(name) << " for joint '" << name << "'";

    set_state(name, get_command(name));
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  drqp_control::a1_16_hardware_interface, hardware_interface::SystemInterface)
