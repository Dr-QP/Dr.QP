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
#include <drqp_a1_16_driver/SerialFactory.h>
#include <drqp_a1_16_driver/XYZrobotServo.h>

#include <cstdint>
#include <exception>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/async_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>
#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>
#include <rclcpp/subscription_base.hpp>

using namespace std::chrono_literals;

class PoseSetter : public rclcpp::Node
{
public:
  PoseSetter() : Node("drqp_pose_setter")
  {
    // declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("device_address", "192.168.0.181:2022");
    declare_parameter("baud_rate", 115200);

    auto subscription_ = this->create_subscription<drqp_interfaces::msg::MultiSyncPositionCommand>(
      "pose_sync", 10, [this](const drqp_interfaces::msg::MultiSyncPositionCommand& msg) {
        try {
          auto pose = drqp_interfaces::msg::MultiSyncPositionCommand{};
          std::unique_ptr<SerialProtocol> servoSerial;
          servoSerial = makeSerialForDevice(get_parameter("device_address").as_string());
          servoSerial->begin(get_parameter("baud_rate").as_int());

          XYZrobotServo servo(*servoSerial, XYZrobotServo::kBroadcastId);

          DynamicSJogCommand sposCmd(msg.positions.size());
          sposCmd.setPlaytime(msg.playtime);
          for (size_t index = 0; index < msg.positions.size(); ++index) {
            auto pos = msg.positions.at(index);
            SJogData data{pos.position, SET_POSITION_CONTROL, pos.id};
            sposCmd.setData(index, data);
          }

          servo.sendJogCommand(sposCmd);
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in pose_sync handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose_sync handler.");
        }
      });
  }

  rclcpp::Subscription<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSetter>());
  rclcpp::shutdown();
  return 0;
}
