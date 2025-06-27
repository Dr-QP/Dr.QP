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

#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <filesystem>
namespace fs = std::filesystem;

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/multi_servo_state.hpp>
#include "drqp_control/JointServoMappings.h"

using namespace std::chrono_literals;

class PoseReader : public rclcpp::Node
{
public:
  PoseReader() : Node("drqp_pose_reader")
  {
    declare_parameter("device_address", "/dev/ttySC0");
    declare_parameter("baud_rate", 115200);
    declare_parameter("first_id", 1);
    declare_parameter("last_id", 18);
    declare_parameter("period_ms", 100);
    declare_parameter("config", "");

    const auto configPath = getConfigPath();
    loadConfig(configPath);

    servoStatesPublisher_ =
      this->create_publisher<drqp_interfaces::msg::MultiServoState>("/servo_states", 10);

    servoSerial_ = makeSerialForDevice(get_parameter("device_address").as_string());
    servoSerial_->begin(get_parameter("baud_rate").as_int());
    const uint8_t firstId = get_parameter("first_id").as_int();
    const uint8_t lastId = get_parameter("last_id").as_int();

    auto timerPeriod = std::chrono::milliseconds(get_parameter("period_ms").as_int());
    timer_ = this->create_wall_timer(timerPeriod, [this, firstId, lastId]() {
      try {
        auto multiServoStates = drqp_interfaces::msg::MultiServoState{};
        multiServoStates.header.stamp = this->get_clock()->now();

        for (uint8_t servoId = firstId; servoId <= lastId; ++servoId) {
          auto startTime = this->get_clock()->now();
          XYZrobotServo servo(*servoSerial_, servoId);

          XYZrobotServoStatus status = servo.readStatus();
          if (servo.isFailed()) {
            RCLCPP_ERROR(
              get_logger(), "Servo %i read status failed %s.", servoId,
              to_string(servo.getLastError()).c_str());
            continue;
          }
          auto servoState = drqp_interfaces::msg::ServoState{};
          auto endTime = this->get_clock()->now();

          servoState.read_duration_microsec = (endTime - startTime).nanoseconds() / 1000;
          servoState.raw.id = servoId;
          servoState.raw.position = status.position;
          servoState.raw.goal = status.posRef;
          servoState.raw.status_error = static_cast<uint8_t>(status.statusError);
          servoState.raw.status_detail = static_cast<uint8_t>(status.statusDetail);
          servoState.raw.torque = status.pwm;

          if (std::optional<JointValues> joint = servoToJoint({servoId, status.position})) {
            servoState.joint_name = joint->name;
            servoState.position_as_radians = joint->position_as_radians;
          }

          multiServoStates.servos.emplace_back(std::move(servoState));
        }
        auto endTime = this->get_clock()->now();
        multiServoStates.read_duration_microsec =
          (endTime - multiServoStates.header.stamp).nanoseconds() / 1000;
        servoStatesPublisher_->publish(multiServoStates);
      } catch (std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception occurred in pose read handler %s", e.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Unknown exception occurred in pose read handler.");
      }
    });
  }

  fs::path getConfigPath() {
    fs::path yamlPath = get_parameter("config").as_string();
    if (yamlPath.empty()) {
      const fs::path packageShareDir = ament_index_cpp::get_package_share_directory(
      "drqp_control");
      yamlPath = packageShareDir / "config" / "drqp.yml";
    }
    if (!fs::exists(yamlPath)) {
      RCLCPP_ERROR(this->get_logger(), "%s could not be found. Exiting.", yamlPath.c_str());
      throw std::runtime_error("Robot config parsing failure.");
    }
    return yamlPath;
  }

  void loadConfig(const fs::path& configPath)
  {
    // bool rcl_parse_yaml_file(const char *file_path, rcl_params_t *params_st)
    rcl_params_t* params = rcl_yaml_node_struct_init(rcutils_get_default_allocator());;
    if (!rcl_parse_yaml_file(configPath.c_str(), params))
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse config file %s", configPath.c_str());
      return;
    }

    // const std::string dimensions_param = "dimensions." + object_name_;
    // rcl_variant_t * dimensions =
    //   rcl_yaml_node_struct_get("dope", dimensions_param.c_str(), dope_params);
    // if (!dimensions->double_array_value) {
    //   RCLCPP_ERROR(
    //     this->get_logger(), "No dimensions parameter found for object name: %s",
    //     object_name_.c_str());
    //   throw std::runtime_error("Parameter parsing failure.");
    // }
  }

  std::unique_ptr<SerialProtocol> servoSerial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<drqp_interfaces::msg::MultiServoState>::SharedPtr servoStatesPublisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseReader>());
  rclcpp::shutdown();
  return 0;
}
