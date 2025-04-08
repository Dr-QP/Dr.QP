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

#include <drqp_interfaces/msg/multi_servo_position_goal.hpp>
#include <rclcpp/subscription_base.hpp>

using namespace std::chrono_literals;

class PoseSetter : public rclcpp::Node
{
public:
  PoseSetter() : Node("drqp_pose_setter")
  {
    declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("baud_rate", 115200);

    servoSerial_ = makeSerialForDevice(get_parameter("device_address").as_string());
    servoSerial_->begin(get_parameter("baud_rate").as_int());

    multiServoPositionGoalSubscription_ = create_subscription<drqp_interfaces::msg::MultiServoPositionGoal>(
      "/servo_goals", 10, [this](const drqp_interfaces::msg::MultiServoPositionGoal& msg) {
        try {
          if (msg.mode == drqp_interfaces::msg::MultiServoPositionGoal::MODE_SYNC) {
            handleSyncPose(msg);
          } else if (msg.mode == drqp_interfaces::msg::MultiServoPositionGoal::MODE_ASYNC) {
            handleAsyncPose(msg);
          } else {
            RCLCPP_ERROR(get_logger(), "Unknown pose mode %i", msg.mode);
          }
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in pose_async handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose_async handler.");
        }
      });
  }

  void handleSyncPose(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    if (msg.goals.empty()) {
      return;
    }

    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

    DynamicSJogCommand sposCmd(msg.goals.size());
    sposCmd.setPlaytime(msg.goals.at(0).playtime);

    for (const auto& pos : msg.goals) {
      sposCmd.at(pos.id) = {pos.position, SET_POSITION_CONTROL, pos.id};  // SJogData
    }

    servo.sendJogCommand(sposCmd);
  }

  void handleAsyncPose(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

    DynamicIJogCommand iposCmd(msg.goals.size());
    for (const auto& pos : msg.goals) {
      iposCmd.at(pos.id) = {pos.position, SET_POSITION_CONTROL, pos.id, pos.playtime}; // IJogData
    }

    servo.sendJogCommand(iposCmd);
  }

  rclcpp::Subscription<drqp_interfaces::msg::MultiServoPositionGoal>::SharedPtr
    multiServoPositionGoalSubscription_;

  std::unique_ptr<SerialProtocol> servoSerial_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSetter>());
  rclcpp::shutdown();
  return 0;
}
