#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <drqp_interfaces/msg/detail/sync_position_command__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <drqp_interfaces/msg/async_position_command.hpp>
#include <drqp_interfaces/msg/multi_async_position_command.hpp>
#include <drqp_interfaces/msg/sync_position_command.hpp>
#include <drqp_interfaces/msg/multi_sync_position_command.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("drqp_a1_16_driver")
    {
      publisher_ = this->create_publisher<drqp_interfaces::msg::SyncPositionCommand>("pose", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = drqp_interfaces::msg::SyncPositionCommand{};
      // message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<drqp_interfaces::msg::SyncPositionCommand>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
