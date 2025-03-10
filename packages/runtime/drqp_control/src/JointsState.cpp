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
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>

using namespace std::chrono_literals;

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

            for (size_t index = 0; index < msg.positions.size(); ++index) {
              auto pos = msg.positions.at(index);
              // SJogData data{pos.position, SET_POSITION_CONTROL, pos.id};

            }

          } catch (std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception occurred in pose_sync handler %s", e.what());
          } catch (...) {
            RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose_sync handler.");
          }
        });
  }

  rclcpp::Subscription<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr
    multiSyncPoseSubscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStateNode>());
  rclcpp::shutdown();
  return 0;
}
