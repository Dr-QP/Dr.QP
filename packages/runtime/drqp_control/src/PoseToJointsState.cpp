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
#include "drqp_control/JointServoMappings.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <drqp_interfaces/msg/multi_position_command.hpp>

class PoseToJointState : public rclcpp::Node
{
public:
  PoseToJointState() : Node("drqp_pose_to_joint_state")
  {
    jointStatesPublisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    multiPoseSubscription_ = this->create_subscription<drqp_interfaces::msg::MultiPositionCommand>(
      "pose", 10, [this](const drqp_interfaces::msg::MultiPositionCommand& msg) {
        try {
          sensor_msgs::msg::JointState jointState;
          jointState.header.stamp = this->get_clock()->now();

          for (size_t index = 0; index < msg.positions.size(); ++index) {
            auto pos = msg.positions.at(index);

            addJointServo(jointState, pos.id, pos.position);
          }

          jointStatesPublisher_->publish(jointState);
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in pose handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose handler.");
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

    const JointParams params = kServoIdToJoint.at(servoId);
    const double positionInRadians = params.ratio * positionToRadians(position);

    jointState.name.push_back(params.jointName);
    jointState.position.push_back(positionInRadians);
  }

  rclcpp::Subscription<drqp_interfaces::msg::MultiPositionCommand>::SharedPtr
    multiPoseSubscription_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPublisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToJointState>());
  rclcpp::shutdown();
  return 0;
}
