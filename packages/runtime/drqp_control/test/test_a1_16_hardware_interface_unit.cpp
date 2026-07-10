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

// Unit tests for drqp_control::a1_16_hardware_interface that exercise the
// hardware interface in isolation from a running ROS 2 system. The serial
// driver is bypassed via the interface's built-in "mock_servo" device address,
// which swaps in MockServo (see drqp_a1_16_driver/MockServo.h) so no real
// serial port is opened. This complements the full-stack integration test in
// test_a1_16_hardware_interface.py by isolating the hardware-interface logic.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"

#include "drqp_control/a1_16_hardware_interface.h"

namespace
{
using drqp_control::a1_16_hardware_interface;
using hardware_interface::CallbackReturn;

// Builds a minimal ros2_control URDF describing a single position-controlled
// joint driven through the mock servo. Any per-joint command-interface <param>
// can be dropped by passing its name in omit_param to exercise error handling.
std::string makeUrdf(const std::string& omit_param = "")
{
  const std::string params[][2] = {
    {"servo_id", "1"},
    {"inverted", "False"},
    {"offset_rads", "0.0"},
    {"max_torque", "1.0"},
    {"min", "-1.5"},
    {"max", "1.5"},
    {"initial_position_rads", "0.25"},
  };

  std::string joint_params;
  for (const auto& kv : params) {
    if (kv[0] == omit_param) {
      continue;
    }
    joint_params += "        <param name=\"" + kv[0] + "\">" + kv[1] + "</param>\n";
  }

  return "<?xml version=\"1.0\"?>\n"
         "<robot name=\"drqp\">\n"
         "  <link name=\"base_link\"/>\n"
         "  <link name=\"link1\"/>\n"
         "  <joint name=\"joint1\" type=\"revolute\">\n"
         "    <parent link=\"base_link\"/>\n"
         "    <child link=\"link1\"/>\n"
         "    <axis xyz=\"0 0 1\"/>\n"
         "    <limit effort=\"1.0\" velocity=\"1.0\" lower=\"-1.5\" upper=\"1.5\"/>\n"
         "  </joint>\n"
         "  <ros2_control name=\"drqp_a1_16\" type=\"system\">\n"
         "    <hardware>\n"
         "      <plugin>drqp_control/a1_16_hardware_interface</plugin>\n"
         "      <param name=\"device_address\">mock_servo</param>\n"
         "      <param name=\"baud_rate\">115200</param>\n"
         "    </hardware>\n"
         "    <joint name=\"joint1\">\n"
         "      <command_interface name=\"position\">\n" +
         joint_params +
         "      </command_interface>\n"
         "      <command_interface name=\"effort\"/>\n"
         "      <state_interface name=\"position\"/>\n"
         "      <state_interface name=\"pwm\"/>\n"
         "    </joint>\n"
         "  </ros2_control>\n"
         "</robot>\n";
}

hardware_interface::HardwareComponentParams makeParams(const std::string& urdf)
{
  const std::vector<hardware_interface::HardwareInfo> infos =
    hardware_interface::parse_control_resources_from_urdf(urdf);
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = infos.at(0);
  params.logger = rclcpp::get_logger("test_a1_16_hardware_interface_unit");
  params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  return params;
}

rclcpp_lifecycle::State inactiveState()
{
  return rclcpp_lifecycle::State(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
}

// A hardware component may spin up an internal node during init(); make sure
// rclcpp is initialised for the duration of the test program.
class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};
const ::testing::Environment* const kRclcppEnv =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);

TEST(A1_16HardwareInterfaceInit, SucceedsWithValidConfig)
{
  a1_16_hardware_interface hw;
  EXPECT_EQ(hw.init(makeParams(makeUrdf())), CallbackReturn::SUCCESS);
}

TEST(A1_16HardwareInterfaceInit, FailsWhenMandatoryParamMissing)
{
  a1_16_hardware_interface hw;
  // servo_id is required to register the joint; its absence must be reported as
  // an ERROR rather than propagating an exception out of on_init.
  EXPECT_EQ(hw.init(makeParams(makeUrdf("servo_id"))), CallbackReturn::ERROR);
}

TEST(A1_16HardwareInterfaceLifecycle, ActivateAndDeactivateSucceed)
{
  a1_16_hardware_interface hw;
  ASSERT_EQ(hw.init(makeParams(makeUrdf())), CallbackReturn::SUCCESS);

  EXPECT_EQ(hw.on_activate(inactiveState()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.on_deactivate(inactiveState()), CallbackReturn::SUCCESS);
}

TEST(A1_16HardwareInterfaceReadWrite, WriteCommandUpdatesPositionState)
{
  a1_16_hardware_interface hw;
  ASSERT_EQ(hw.init(makeParams(makeUrdf())), CallbackReturn::SUCCESS);

  // Allocate the state/command interface storage the callbacks operate on.
  hw.on_export_state_interfaces();
  hw.on_export_command_interfaces();

  ASSERT_EQ(hw.on_configure(inactiveState()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw.on_activate(inactiveState()), CallbackReturn::SUCCESS);

  const double commanded_position = 0.5;
  hw.set_command("joint1/position", commanded_position);
  hw.set_command("joint1/effort", 1.0);  // effort >= 0.1 keeps torque on

  const rclcpp::Time time{};
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.01);
  ASSERT_EQ(hw.write(time, period), hardware_interface::return_type::OK);

  // write() mirrors the commanded position into the position state interface
  // (round-tripped through the servo unit conversion) for torque-on servos.
  EXPECT_NEAR(hw.get_state<double>("joint1/position"), commanded_position, 0.02);

  // A subsequent read() must observe the goal stored by the mock servo.
  ASSERT_EQ(hw.read(time, period), hardware_interface::return_type::OK);
  EXPECT_NEAR(hw.get_state<double>("joint1/position"), commanded_position, 0.02);
}

}  // namespace
