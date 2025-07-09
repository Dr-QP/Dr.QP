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
#include <drqp_serial/SerialFactory.h>
#include <drqp_a1_16_driver/XYZrobotServo.h>

#include <exception>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>

#include <std_msgs/msg/empty.hpp>

#include <drqp_interfaces/msg/multi_servo_position_goal.hpp>
#include <drqp_interfaces/msg/multi_servo_state.hpp>
#include <drqp_interfaces/msg/torque_on.hpp>

#include "drqp_control/RobotConfig.h"
#include "drqp_control/DrQp.h"

using namespace std::chrono_literals;

class PoseSetter : public rclcpp::Node
{
public:
  PoseSetter() : Node("drqp_pose_setter"), robotConfig_(this)
  {
    robotConfig_.loadConfig();

    servoSerial_ = makeSerialForDevice(get_parameter("device_address").as_string());
    servoSerial_->begin(get_parameter("baud_rate").as_int());

    servoStatesPublisher_ =
      this->create_publisher<drqp_interfaces::msg::MultiServoState>("/servo_states", 10);

    multiServoPositionGoalSubscription_ =
      create_subscription<drqp_interfaces::msg::MultiServoPositionGoal>(
        "/servo_goals", 10, [this](const drqp_interfaces::msg::MultiServoPositionGoal& msg) {
          try {
            if (!torqueIsOn_) {
              RCLCPP_INFO(get_logger(), "Torque is off, turning on to avoid jerking.");
              torqueOn();
            }

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

    torqueOnSubscription_ = create_subscription<drqp_interfaces::msg::TorqueOn>(
      "/servo_torque_on", 10, [this](const drqp_interfaces::msg::TorqueOn msg) {
        try {
          if (msg.torque_on.empty()) {
            RCLCPP_ERROR(get_logger(), "Torque on message has to have at least 1 element");
            return;
          }
          if (msg.joint_names.empty()) {
            if (msg.torque_on.size() != 1) {
              RCLCPP_ERROR(get_logger(), "Torque on message has to have only 1 element");
              return;
            }
            if (msg.torque_on[0]) {
              RCLCPP_INFO(get_logger(), "Turning torque on for all servos");
              torqueOn();
            } else {
              RCLCPP_INFO(get_logger(), "Turning torque off for all servos");
              torqueOff();
            }
          } else {
            if (msg.torque_on.size() != msg.joint_names.size()) {
              RCLCPP_ERROR(
                get_logger(), "Torque on message has to have the same number of elements as joint names");
              return;
            }
            for (size_t i = 0; i < msg.joint_names.size(); ++i) {
              if (auto servoValues = robotConfig_.jointToServo({msg.joint_names[i], 0.0})) {
                if (msg.torque_on[i]) {
                  RCLCPP_DEBUG(get_logger(), "Turning torque on for %s", msg.joint_names[i].c_str());
                  torqueOn(servoValues->id);
                } else {
                  RCLCPP_DEBUG(get_logger(), "Turning torque off for %s", msg.joint_names[i].c_str());
                  torqueOff(servoValues->id);
                }
              }
            }
          }

        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in kill_switch handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in kill_switch handler.");
        }
      });

    rebootServoSubscription_ = create_subscription<std_msgs::msg::Empty>(
      "/servo_reboot", 10, [this](const std_msgs::msg::Empty) {
        try {
          rebootServos();
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in kill_switch handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in kill_switch handler.");
        }
      });
  }

  ~PoseSetter()
  {
    torqueOff();
  }

  void torqueOn(const uint8_t servoId = XYZrobotServo::kBroadcastId)
  {
    XYZrobotServo servo(*servoSerial_, servoId);
    servo.torqueOn();

    torqueIsOn_ = true;
  }

  void torqueOff(const uint8_t servoId = XYZrobotServo::kBroadcastId)
  {
    XYZrobotServo servo(*servoSerial_, servoId);
    servo.torqueOff();

    torqueIsOn_ = false;
  }

  void rebootServos()
  {
    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);
    servo.reboot();
  }

  void handleSyncPose(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    if (msg.goals.empty()) {
      return;
    }

    XYZrobotServo servo(*servoSerial_, XYZrobotServo::kBroadcastId);

    DynamicSJogCommand sposCmd(msg.goals.size());
    sposCmd.setPlaytime(millisToPlaytime(msg.goals.at(0).playtime_ms));

    for (size_t index = 0; index < msg.goals.size(); ++index) {
      auto pos = msg.goals.at(index);
      auto servoValues = robotConfig_.jointToServo({pos.joint_name, pos.position_as_radians});
      if (!servoValues) {
        RCLCPP_ERROR(get_logger(), "Unknown joint name %s", pos.joint_name.c_str());
        continue;
      }
      sposCmd.at(index) = {servoValues->position, SET_POSITION_CONTROL, servoValues->id};
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
      auto servoValues = robotConfig_.jointToServo({pos.joint_name, pos.position_as_radians});
      if (!servoValues) {
        RCLCPP_ERROR(get_logger(), "Unknown joint name %s", pos.joint_name.c_str());
        continue;
      }
      iposCmd.at(index) = {
        servoValues->position, SET_POSITION_CONTROL, servoValues->id,
        millisToPlaytime(pos.playtime_ms)};
    }

    servo.sendJogCommand(iposCmd);
    publishServoStates(msg);
  }

  void publishServoStates(const drqp_interfaces::msg::MultiServoPositionGoal& msg)
  {
    auto multiServoStates = drqp_interfaces::msg::MultiServoState{};
    for (const auto& posGoal : msg.goals) {
      auto servoState = drqp_interfaces::msg::ServoState{};

      servoState.joint_name = posGoal.joint_name;
      servoState.position_as_radians = posGoal.position_as_radians;
      if (
        auto servoValues =
          robotConfig_.jointToServo({posGoal.joint_name, posGoal.position_as_radians})) {
        servoState.raw.id = servoValues->id;
        servoState.raw.position = servoValues->position;
      }

      multiServoStates.servos.emplace_back(servoState);
    }
    servoStatesPublisher_->publish(multiServoStates);
  }

  RobotConfig robotConfig_;

  rclcpp::Publisher<drqp_interfaces::msg::MultiServoState>::SharedPtr servoStatesPublisher_;

  rclcpp::Subscription<drqp_interfaces::msg::MultiServoPositionGoal>::SharedPtr
    multiServoPositionGoalSubscription_;

  bool torqueIsOn_ = false;
  rclcpp::Subscription<drqp_interfaces::msg::TorqueOn>::SharedPtr torqueOnSubscription_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr rebootServoSubscription_;

  std::unique_ptr<SerialProtocol> servoSerial_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseSetter>());
  rclcpp::shutdown();
  return 0;
}
