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

#include <exception>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/multi_servo_position_goal.hpp>
#include <drqp_interfaces/msg/multi_servo_state.hpp>
#include <drqp_interfaces/msg/kill_switch.hpp>
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

    servoStatesPublisher_ =
      this->create_publisher<drqp_interfaces::msg::MultiServoState>("/servo_states", 10);

    multiServoPositionGoalSubscription_ =
      create_subscription<drqp_interfaces::msg::MultiServoPositionGoal>(
        "/servo_goals", 10, [this](const drqp_interfaces::msg::MultiServoPositionGoal& msg) {
          try {
            if (killModeActive_) {
              RCLCPP_DEBUG(get_logger(), "Kill switch is on, not setting pose");
              return;
            } else if (rclcpp::Time(msg.header.stamp) < unkillTimestamp) {
              RCLCPP_DEBUG(get_logger(), "Message is older than kill switch, not setting pose");
              return;
            }

            auto multiServoStates = drqp_interfaces::msg::MultiServoState{};
            if (msg.mode == drqp_interfaces::msg::MultiServoPositionGoal::MODE_SYNC) {
              handleSyncPose(msg);
            } else if (msg.mode == drqp_interfaces::msg::MultiServoPositionGoal::MODE_ASYNC) {
              handleAsyncPose(msg);
            } else {
              RCLCPP_ERROR(get_logger(), "Unknown pose mode %i", msg.mode);
            }
          } catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in servo_goals handler %s", e.what());
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unknown exception occurred in servo_goals handler.");
          }
        });

    killSwitchSubscription_ = create_subscription<drqp_interfaces::msg::KillSwitch>(
      "/kill_switch", 10, [this](const drqp_interfaces::msg::KillSwitch& msg) {
        try {
          if (!killModeActive_) {
            killModeActive_ = true;

            torqueOff();

            RCLCPP_INFO(get_logger(), "Kill mode activated");
          } else {
            killModeActive_ = false;
            unkillTimestamp = msg.header.stamp;

            torqueOn();

            RCLCPP_INFO(get_logger(), "Kill mode deactivated");
          }
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in kill_switch handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in kill_switch handler.");
        }
      });

    // torqueOn();
  }

  ~PoseSetter()
  {
    // torqueOff();
  }

  void torqueOn()
  {
    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);
    servo.torqueOn();
  }

  void torqueOff()
  {
    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);
    servo.torqueOff();
  }

  void handleSyncPose(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    if (msg.goals.empty()) {
      return;
    }

    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

    DynamicSJogCommand sposCmd(msg.goals.size());
    sposCmd.setPlaytime(msg.goals.at(0).playtime);

    for (size_t index = 0; index < msg.goals.size(); ++index) {
      auto pos = msg.goals.at(index);
      sposCmd.at(index) = {pos.position, SET_POSITION_CONTROL, pos.id};
    }

    servo.sendJogCommand(sposCmd);
    publishServoStates(msg);
  }

  void handleAsyncPose(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

    DynamicIJogCommand iposCmd(msg.goals.size());
    for (size_t index = 0; index < msg.goals.size(); ++index) {
      auto pos = msg.goals.at(index);
      iposCmd.at(index) = {pos.position, SET_POSITION_CONTROL, pos.id, pos.playtime};
    }

    servo.sendJogCommand(iposCmd);
    publishServoStates(msg);
  }

  void publishServoStates(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    auto multiServoStates = drqp_interfaces::msg::MultiServoState{};
    for (const auto& posGoal : msg.goals) {
      auto servoState = drqp_interfaces::msg::ServoState{};
      servoState.id = posGoal.id;
      servoState.position = posGoal.position;
      multiServoStates.servos.emplace_back(servoState);
    }
    servoStatesPublisher_->publish(multiServoStates);
  }

  rclcpp::Publisher<drqp_interfaces::msg::MultiServoState>::SharedPtr servoStatesPublisher_;

  rclcpp::Subscription<drqp_interfaces::msg::MultiServoPositionGoal>::SharedPtr
    multiServoPositionGoalSubscription_;

  rclcpp::Subscription<drqp_interfaces::msg::KillSwitch>::SharedPtr killSwitchSubscription_;
  bool killModeActive_ = false;
  rclcpp::Time unkillTimestamp = this->get_clock()->now();

  std::unique_ptr<SerialProtocol> servoSerial_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSetter>());
  rclcpp::shutdown();
  return 0;
}
