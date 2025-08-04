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

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <drqp_control/RobotConfig.h>
#include <drqp_serial/SerialProtocol.h>

namespace drqp_control
{

class a1_16_hardware_interface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(a1_16_hardware_interface);

  a1_16_hardware_interface();
  ~a1_16_hardware_interface();

  /**
   * @brief Initialization callback for hardware interface.
   * @param info Hardware information for the system.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * @brief Callback for activating the hardware interface.
   * @param previous_state Previous lifecycle state.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Callback for deactivating the hardware interface.
   * @param previous_state Previous lifecycle state.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Callback for configuring the hardware interface.
   * @param previous_state Previous lifecycle state.
   * @return Callback return indicating success or error.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Reads data from the hardware.
   * @param time Current time.
   * @param period Duration since the last read.
   * @return Hardware interface return type indicating success or error.
   */
  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /**
   * @brief Writes data to the hardware.
   * @param time Current time.
   * @param period Duration since the last write.
   * @return Hardware interface return type indicating success or error.
   */
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  RobotConfig robotConfig_;
  std::unique_ptr<SerialProtocol> servoSerial_;
};
}  // namespace drqp_control
