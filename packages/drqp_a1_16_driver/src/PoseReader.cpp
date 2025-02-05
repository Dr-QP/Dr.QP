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

#include <chrono>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/async_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>
#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>

using namespace std::chrono_literals;

class PoseReader : public rclcpp::Node
{
public:
  PoseReader() : Node("drqp_pose_reader")
  {
    // declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("device_address", "192.168.0.181:2022");
    declare_parameter("baud_rate", 115200);
    declare_parameter("first_id", 1);
    declare_parameter("last_id", 18);
    declare_parameter("period_ms", 500);

    subscription_ =
      this->create_publisher<drqp_interfaces::msg::MultiSyncPositionCommand>("pose", 10);

    auto timerPeriod = std::chrono::milliseconds(get_parameter("period_ms").as_int());
    timer_ = this->create_wall_timer(timerPeriod, [this, timerPeriod]() {
      try {
        auto pose = drqp_interfaces::msg::MultiSyncPositionCommand{};
        pose.playtime = toPlaytime(timerPeriod);
        std::unique_ptr<SerialProtocol> servoSerial;
        servoSerial = makeSerialForDevice(get_parameter("device_address").as_string());
        servoSerial->begin(get_parameter("baud_rate").as_int());

        const uint8_t firstId = get_parameter("first_id").as_int();
        const uint8_t lastId = get_parameter("last_id").as_int();

        for (uint8_t servoId = firstId; servoId <= lastId; ++servoId) {
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
        subscription_->publish(pose);
      } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception occurred in pose read handler %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose read handler.");
      }
    });
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseReader>());
  rclcpp::shutdown();
  return 0;
}
