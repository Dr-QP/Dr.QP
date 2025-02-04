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
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/async_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>
#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>

using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node
{
public:
  DriverNode() : Node("drqp_a1_16_driver")
  {
    // declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("device_address", "192.168.0.181:2022");

    publisher_ = this->create_publisher<drqp_interfaces::msg::MultiSyncPositionCommand>("pose", 10);

    timer_ = this->create_wall_timer(500ms, [this]() {
      auto pose = drqp_interfaces::msg::MultiSyncPositionCommand{};
      std::unique_ptr<SerialProtocol> servoSerial;
      servoSerial = makeSerialForDevice(get_parameter("device_address").as_string());
      for (uint8_t servoId = 1; servoId <= 18; ++servoId) {
        XYZrobotServo servo(*servoSerial, servoId);

        XYZrobotServoStatus status = servo.readStatus();
        if (servo.isFailed()) {
          continue;
        }
        drqp_interfaces::msg::SyncPositionCommand pos;
        pos.id = servoId;
        pos.position = status.position;
        pose.positions.push_back(pos);
      }
      publisher_->publish(pose);
    });
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverNode>());
  rclcpp::shutdown();
  return 0;
}
