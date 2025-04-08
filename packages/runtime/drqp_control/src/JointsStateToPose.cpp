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

#include <exception>

#include "drqp_control/JointServoMappings.h"
#include "drqp_interfaces/msg/servo_position_goal.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <drqp_interfaces/msg/multi_servo_position_goal.hpp>

class JointsStateToPoseNode : public rclcpp::Node
{
public:
  JointsStateToPoseNode() : Node("drqp_joint_state_to_pose")
  {
    servoGoalsPublisher_ =
      this->create_publisher<drqp_interfaces::msg::MultiServoPositionGoal>("servo_goals", 10);

    jointStateSubscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [this](const sensor_msgs::msg::JointState& msg) {
        try {
          drqp_interfaces::msg::MultiServoPositionGoal pose;
          pose.mode = drqp_interfaces::msg::MultiServoPositionGoal::MODE_ASYNC;

          for (size_t index = 0; index < msg.name.size(); ++index) {
            auto pos = msg.name.at(index);

            if (kJointToServoId.count(pos) == 0) {
              RCLCPP_ERROR(get_logger(), "Skipping unknown joint name %s", pos.c_str());
              continue;
            }

            const ServoParams params = kJointToServoId.at(pos);
            const uint16_t position = radiansToPosition(params.ratio * msg.position[index]);

            drqp_interfaces::msg::ServoPositionGoal goalCmd;
            goalCmd.id = params.id;
            goalCmd.position = position;
            goalCmd.playtime = 0;
            pose.goals.emplace_back(std::move(goalCmd));
          }

          servoGoalsPublisher_->publish(pose);
        } catch (std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Exception occurred in pose handler %s", e.what());
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose handler.");
        }
      });
  }

private:
  rclcpp::Publisher<drqp_interfaces::msg::MultiServoPositionGoal>::SharedPtr
    servoGoalsPublisher_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointsStateToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
