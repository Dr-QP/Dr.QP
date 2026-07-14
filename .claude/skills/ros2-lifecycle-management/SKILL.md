---
name: ros2-lifecycle-management
description: Implement, transition, and diagnose ROS 2 managed lifecycle nodes. Use when a node needs configure, activate, deactivate, cleanup, shutdown, or error-processing behavior; use a state-machine skill for a general application finite-state machine.
---

# ROS 2 Lifecycle Management

Use lifecycle nodes when a ROS node must reserve resources before operation
and release them predictably. This skill is specifically for ROS managed-node
semantics, not a generic behavior state machine; route ordinary C++ state
machines to [create-state-machine-cpp](../create-state-machine-cpp/SKILL.md).

## Run and inspect transitions reliably

Run commands from the workspace root through the ROS wrapper:

```bash
scripts/with-ros-env.sh ros2 lifecycle nodes
scripts/with-ros-env.sh ros2 lifecycle get /<node_name>
scripts/with-ros-env.sh ros2 lifecycle list /<node_name> -a
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> configure
```

In Codex, request sandbox escalation for a ROS command that starts a node or
uses graph sockets. If the wrapper cannot source ROS, follow the Docker then
Codespaces escalation route in
[ros2-workspace-build](../ros2-workspace-build/SKILL.md), rather than retrying
on the host. For a system already running, use
[ros2-diagnostics](../ros2-diagnostics/SKILL.md) to inspect logs, the graph,
and parameters.

## Model the lifecycle correctly

The primary states are **Unconfigured**, **Inactive**, **Active**, and
**Finalized**. Requests pass through transitional states (such as configuring,
activating, deactivating, cleaning up, and shutting down). The normal control
path is:

```text
Unconfigured --configure--> Inactive --activate--> Active
     ^                              |                 |
     |                              |                 |
     +-----------cleanup------------+      deactivate--+
                                      \--> Inactive
Any primary state --shutdown--> Finalized
```

Do not assume every failed command returns a node to its old state. A callback
that returns `FAILURE` rejects the requested transition; a callback that
returns `ERROR` enters error processing and invokes `on_error`. Implement
`on_error` when recovery matters, log the original cause, and use
`ros2 lifecycle get` plus node logs to confirm the resulting state. An error
handler's return value determines whether the node can return to a recoverable
state or is finalized; consult the lifecycle API for the exact policy of the
ROS release in use.

## Operate a node manually

Transition only when the current state permits it. A simple controlled session
is:

```bash
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> configure
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> activate
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> deactivate
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> cleanup
scripts/with-ros-env.sh ros2 lifecycle set /<node_name> shutdown
```

For dependent nodes, configure dependencies before consumers, verify each
state, then activate in the same order. Keep coordination policy in a launch
or supervisor design; use [ros2-launch-management](../ros2-launch-management/SKILL.md)
when creating that launch setup rather than copying transition shell loops.

## Implement a C++ lifecycle node

Declare every parameter before reading it and make every transition safe when
resources are absent. This minimal executable has the required includes,
dependencies, timer guards, and error return paths:

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  MyLifecycleNode()
  : LifecycleNode("my_lifecycle_node")
  {
    declare_parameter<std::string>("robot_name", "robot");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    robot_name_ = get_parameter("robot_name").as_string();
    if (robot_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "robot_name must not be empty");
      return CallbackReturn::FAILURE;
    }
    publisher_ = create_publisher<std_msgs::msg::String>("output", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!publisher_) {
      RCLCPP_ERROR(get_logger(), "Publisher was not configured");
      return CallbackReturn::FAILURE;
    }
    publisher_->on_activate();
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&MyLifecycleNode::publish_message, this));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    if (publisher_) {
      publisher_->on_deactivate();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_ERROR(get_logger(), "Lifecycle transition entered error processing");
    timer_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void publish_message()
  {
    if (!publisher_ || !publisher_->is_activated()) {
      return;
    }
    std_msgs::msg::String message;
    message.data = "Hello from " + robot_name_;
    publisher_->publish(message);
  }

  std::string robot_name_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyLifecycleNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
```

Add matching runtime dependencies in `package.xml` and link the executable:

```xml
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>std_msgs</depend>
```

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_lifecycle_node src/my_lifecycle_node.cpp)
ament_target_dependencies(my_lifecycle_node
  rclcpp rclcpp_lifecycle std_msgs)
install(TARGETS my_lifecycle_node DESTINATION lib/${PROJECT_NAME})
```

Use [add-test-file-cpp](../add-test-file-cpp/SKILL.md) for C++ transition-unit
tests, and [launch-testing](../launch-testing/SKILL.md) when testing a node
through its launch description. Build and run only the target package using
[ros2-workspace-build](../ros2-workspace-build/SKILL.md) and
[ros2-workspace-testing](../ros2-workspace-testing/SKILL.md).

## Related skills

- [ros2-launch-management](../ros2-launch-management/SKILL.md)
- [ros2-diagnostics](../ros2-diagnostics/SKILL.md)
- [add-test-file-cpp](../add-test-file-cpp/SKILL.md)
- [launch-testing](../launch-testing/SKILL.md)
- [ros2-workspace-build](../ros2-workspace-build/SKILL.md)
- [ros2-workspace-testing](../ros2-workspace-testing/SKILL.md)
