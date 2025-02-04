#include <drqp_serial/SerialProtocol.h>
#include <drqp_serial/TcpSerial.h>
#include <drqp_a1_16_driver/XYZrobotServo.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/async_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>
#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("drqp_a1_16_driver")
  {
    publisher_ = this->create_publisher<drqp_interfaces::msg::MultiSyncPositionCommand>("pose", 10);

    timer_ = this->create_wall_timer(500ms, [this]() {
      auto pose = drqp_interfaces::msg::MultiSyncPositionCommand{};
      std::unique_ptr<SerialProtocol> servoSerial;
      servoSerial = std::make_unique<TcpSerial>("192.168.0.181", "2022");
      for (uint8_t servoId = 1; servoId <= 18; ++servoId) {
        XYZrobotServo servo(*servoSerial, servoId);

        XYZrobotServoStatus status = servo.readStatus();
        if (servo.isFailed()) {
        }
        drqp_interfaces::msg::SyncPositionCommand pos;
        pos.id = servoId;
        pos.position = status.position;
        pose.positions.push_back(pos);
      }
      publisher_->publish(pose);
    });
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<drqp_interfaces::msg::MultiSyncPositionCommand>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
