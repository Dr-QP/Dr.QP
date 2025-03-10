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

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <algorithm>

#include "drqp_control/DrQp.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <drqp_interfaces/msg/multi_sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>

struct Params
{
  std::string name;
  double ratio = 1.;
};

static const auto kServoIdToJoint = []() {
  std::unordered_map<uint8_t, Params> result;

  const std::string kRight = "right";
  const std::array<const char*, kServosPerLeg> kJointNames = {"-coxa", "-femur", "-tibia"};
  for (const auto& leg : kAllLegServoIds) {
    int jointNameIndex = 0;
    for (const int servoId : leg) {
      const std::string legName = legNameForServo(servoId);
      std::string jointName = "dr_qp/" + legName + kJointNames[jointNameIndex];
      std::replace(jointName.begin(), jointName.end(), '-', '_');

      const bool isRight =
        std::find_end(legName.begin(), legName.end(), kRight.begin(), kRight.end()) !=
        legName.end();
      result[servoId] = {jointName, (isRight ? -1. : 1.) / 1023.};
      ++jointNameIndex;
    }
  }
  return result;
}();

class JointStateNode : public rclcpp::Node
{
public:
  JointStateNode() : Node("drqp_joint_state")
  {
    // declare_parameter("period_ms", 500);

    joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    multiSyncPoseSubscription_ =
      this->create_subscription<drqp_interfaces::msg::MultiSyncPositionCommand>(
        "pose", 10, [this](const drqp_interfaces::msg::MultiSyncPositionCommand& msg) {
          try {
            sensor_msgs::msg::JointState jointState;
            jointState.header.stamp = this->get_clock()->now();

            for (size_t index = 0; index < msg.positions.size(); ++index) {
              auto pos = msg.positions.at(index);

              addJointServo(jointState, pos.id, pos.position);
            }

            joint_states_publisher_->publish(jointState);
          } catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in pose handler %s", e.what());
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose handler.");
          }
        });

    multiAsyncPoseSubscription_ =
      create_subscription<drqp_interfaces::msg::MultiAsyncPositionCommand>(
        "pose_async", 10, [this](const drqp_interfaces::msg::MultiAsyncPositionCommand& msg) {
          try {
            sensor_msgs::msg::JointState jointState;
            jointState.header.stamp = this->get_clock()->now();

            for (size_t index = 0; index < msg.positions.size(); ++index) {
              auto pos = msg.positions.at(index);

              addJointServo(jointState, pos.id, pos.position);
            }

            joint_states_publisher_->publish(jointState);
          } catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in pose_async handler %s", e.what());
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose_async handler.");
          }
        });
  }

private:
  void addJointServo(sensor_msgs::msg::JointState& jointState, ServoId servoId, uint16_t position)
  {
    if (kServoIdToJoint.count(servoId) == 0) {
      RCLCPP_ERROR(get_logger(), "Skipping unknown servo id %i", servoId);
      return;
    }

    const Params params = kServoIdToJoint.at(servoId);
    const double positionAsRatio = position * params.ratio;
    // Position => Radians
    // 0 => -Pi
    // 512 => 0
    // 1023 => Pi
    const double positionInRadians = positionAsRatio * (2 * 3.14) - 3.14;

    jointState.name.push_back(params.name);
    jointState.position.push_back(positionInRadians);
  }

  rclcpp::Subscription<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr
    multiSyncPoseSubscription_;

  rclcpp::Subscription<drqp_interfaces::msg::MultiAsyncPositionCommand>::SharedPtr
    multiAsyncPoseSubscription_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateNode>());
  rclcpp::shutdown();
  return 0;
}
