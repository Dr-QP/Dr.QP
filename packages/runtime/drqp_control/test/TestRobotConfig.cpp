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

#include <catch_amalgamated.hpp>
#include <catch_ros2/catch.hpp>

#include "drqp_control/RobotConfig.h"

SCENARIO("ROS node")
{
  rclcpp::init(0, nullptr);
  GIVEN("Robot config")
  {
    rclcpp::Node node("test_robot_config");
    NodeRobotConfig robotConfig(&node);

    THEN("Config parameter should be declared")
    {
      CHECK_NOTHROW(node.get_parameter("config").as_string());
    }

    THEN("Default config file should be found")
    {
      CHECK_FALSE(robotConfig.getConfigPath().empty());
      CHECK(fs::exists(robotConfig.getConfigPath()));
    }

    THEN("Default config file should be loaded")
    {
      CHECK_NOTHROW(robotConfig.loadConfig());
    }

    WHEN("Config file is provided")
    {
      node.set_parameter(rclcpp::Parameter("config", "path that does not exist"));

      THEN("Config file should not be found")
      {
        CHECK_THROWS(robotConfig.getConfigPath());
      }
    }

    WHEN("Config file exists and is loaded")
    {
      REQUIRE_NOTHROW(
        robotConfig.loadConfig(TEST_DATA_DIR_IN_SOURCE_TREE "/test_robot_config.yml"));

      THEN("Parameters should be declared")
      {
        CHECK(node.get_parameter("device_address").as_string() == "/dev/ttySC0");
        CHECK(node.get_parameter("baud_rate").as_int() == 115200);
      }

      THEN("Joint to servo mapping should work with default limits")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_coxa", M_PI};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 1);
        CHECK(servo.position == 1023);
      }

      THEN("Joint to servo mapping should work within limits")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_coxa_with_limits", 1.0};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 10);
        CHECK(servo.position == 674);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(1.0, 0.01));
      }

      using double_limits = std::numeric_limits<double>;
      THEN("Joint to servo mapping should have upper clamping.")
      {
        const double outOfBoundUpper = GENERATE(3.0, 100000.0, double_limits::infinity());
        CAPTURE(outOfBoundUpper);
        RobotConfig::JointValues joint{"test_robot/left_front_coxa_with_limits", outOfBoundUpper};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 10);
        CHECK(servo.position == 775);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(1.62, 0.01));
      }

      THEN("Joint to servo mapping should have lower clamping")
      {
        const double outOfBoundLower = GENERATE(
          -3.0, -100000.0, -double_limits::infinity(), double_limits::quiet_NaN(),
          double_limits::signaling_NaN());
        CAPTURE(outOfBoundLower);
        RobotConfig::JointValues joint{"test_robot/left_front_coxa_with_limits", outOfBoundLower};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 10);
        CHECK(servo.position == 348);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(-1.0, 0.01));
      }

      THEN("Servo to joint mapping should work")
      {
        RobotConfig::ServoValues servo{1, 1023};
        RobotConfig::JointValues joint = robotConfig.servoToJoint(servo).value();
        CHECK(joint.name == "test_robot/left_front_coxa");
        CHECK_THAT(joint.position_as_radians, Catch::Matchers::WithinAbs(M_PI, 0.01));
      }

      THEN("Inverted servo should work")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_tibia", M_PI};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 18);
        CHECK(servo.position == 0);
      }

      THEN("Inverted servo should work within limits")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_tibia_with_limits", 1.0};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 18);
        CHECK(servo.position == 348);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(1.0, 0.01));
      }

      THEN("Inverted servo should have max clamping")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_tibia_with_limits", 3.0};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 18);
        CHECK(servo.position == 247);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(1.62, 0.01));
      }

      THEN("Inverted servo should have min clamping")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_tibia_with_limits", -3.0};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 18);
        CHECK(servo.position == 674);

        RobotConfig::JointValues joint2 = robotConfig.servoToJoint(servo).value();
        CHECK_THAT(joint2.position_as_radians, Catch::Matchers::WithinAbs(-1.0, 0.01));
      }

      THEN("Offset should work from angle")
      {
        RobotConfig::JointValues joint{"test_robot/left_front_femur", 0.0};
        RobotConfig::ServoValues servo = robotConfig.jointToServo(joint).value();
        CHECK(servo.id == 2);
        CHECK(servo.position == 544);
      }

      THEN("Offset should work to angle")
      {
        RobotConfig::ServoValues servo{2, 544};
        RobotConfig::JointValues joint = robotConfig.servoToJoint(servo).value();
        CHECK(joint.name == "test_robot/left_front_femur");
        CHECK_THAT(joint.position_as_radians, Catch::Matchers::WithinAbs(0.0, 0.01));
      }

      THEN("getJointNames should return all joints defined in config")
      {
        CHECK(robotConfig.numServos() == 5);

        const auto jointNames = robotConfig.getJointNames();
        CHECK(jointNames.size() == 5);
        CHECK_THAT(jointNames, Catch::Matchers::Contains("test_robot/left_front_coxa"));
        CHECK_THAT(jointNames, Catch::Matchers::Contains("test_robot/left_front_coxa_with_limits"));
        CHECK_THAT(jointNames, Catch::Matchers::Contains("test_robot/left_front_femur"));
        CHECK_THAT(jointNames, Catch::Matchers::Contains("test_robot/left_front_tibia"));
        CHECK_THAT(
          jointNames, Catch::Matchers::Contains("test_robot/left_front_tibia_with_limits"));
      }

      THEN("Invalid joint name should return nullopt")
      {
        RobotConfig::JointValues joint{"test_robot/invalid_joint", 0.0};
        CHECK_FALSE(robotConfig.jointToServo(joint));
      }
      THEN("Invalid servo id should return nullopt")
      {
        RobotConfig::ServoValues servo{99, 0};
        CHECK_FALSE(robotConfig.servoToJoint(servo));
      }
    }

    WHEN("Config file is invalid")
    {
      THEN("Loading should throw")
      {
        CHECK_THROWS(
          robotConfig.loadConfig(TEST_DATA_DIR_IN_SOURCE_TREE "/invalid_robot_config.yml"));
        CHECK_THROWS(robotConfig.loadConfig(TEST_DATA_DIR_IN_SOURCE_TREE "/non_robot_config.yml"));
        CHECK_THROWS(
          robotConfig.loadConfig(TEST_DATA_DIR_IN_SOURCE_TREE "/non_existing_config.yml"));
      }
    }
  }

  rclcpp::shutdown();
}
