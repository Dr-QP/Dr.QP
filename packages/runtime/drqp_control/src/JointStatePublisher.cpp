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

#include "drqp_control/JointStatePublisher.h"

JointStatePublisher::JointStatePublisher(rclcpp::Node& node)
{
  jointStatesPublisher_ = node.create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
}

void JointStatePublisher::publish(rclcpp::Node& node, const drqp_interfaces::msg::MultiServoState& servoStates)
{
  try {
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = node.get_clock()->now();

    for (const auto& servoState : servoStates.servos) {
      jointState.name.push_back(servoState.name);
      jointState.position.push_back(servoState.position_as_radians);
    }

    jointStatesPublisher_->publish(jointState);
  } catch (std::exception& e) {
    RCLCPP_ERROR(node.get_logger(), "Exception occurred in pose handler %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(node.get_logger(), "Unknown exception occurred in pose handler.");
  }
}
