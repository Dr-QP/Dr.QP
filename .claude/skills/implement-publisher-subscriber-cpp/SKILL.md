---
name: implement-publisher-subscriber-cpp
description: Implement ROS 2 C++ rclcpp publisher, subscriber, or paired nodes with explicit message dependencies and supported QoS choices. Use for topic-based C++ communication; use launch-testing when verification must start ROS processes together.
---

# Implement C++ Publisher-Subscriber Nodes

Implement a single-purpose publisher, subscriber, or paired node in an
existing C++ package. Keep message conversion and domain logic testable outside
callbacks; callbacks should validate input, update state, and publish or
schedule bounded work.

Use [implement-publisher-subscriber-python](../implement-publisher-subscriber-python/)
for Python nodes. Use [launch-testing](../launch-testing/) when the result
needs a multi-process or launch-level integration test.

## Gather inputs

Require the package name, node name, message type, topic name, node role, and
QoS profile. For a publisher also require a publishing trigger (timer, input
callback, or explicit API) and a positive timer rate when a timer is used.
Add the message package and `rclcpp` as manifest dependencies; a custom message
package is a direct dependency of both the manifest and target.

Accept only these QoS profile names:

| Profile | rclcpp construction | Intended use |
| --- | --- | --- |
| `default` | `rclcpp::QoS(10)` | General reliable, volatile topic |
| `reliable` | `rclcpp::QoS(10).reliable()` | Delivery preferred over latency |
| `best_effort` | `rclcpp::QoS(10).best_effort()` | Loss-tolerant streaming |
| `sensor_data` | `rclcpp::SensorDataQoS()` | Sensor streams |
| `services` | `rclcpp::ServicesQoS()` | Service-like request/reply topics only |
| `parameters` | `rclcpp::ParametersQoS()` | ROS parameter-event style traffic only |
| `system_default` | `rclcpp::SystemDefaultsQoS()` | Defer policy to the RMW implementation |

Reject any other value. Do not choose `services` or `parameters` for ordinary
application topics merely because their reliability settings look suitable.
Publisher and subscriber endpoints must use compatible reliability and
durability policies; warn before creating a known-incompatible pair.

## Implement and integrate

Place the node implementation at `src/<node_name>_node.cpp`. Derive from
`rclcpp::Node`, create the publisher/subscription in the constructor, retain
the publisher, subscription, and timer as members, and use a captured
`std::shared_ptr<const MessageT>` in subscription callbacks. Give executable
targets a small `main` that calls `rclcpp::init`, spins the node, and calls
`rclcpp::shutdown` after spinning.

In `CMakeLists.txt`, add the executable, declare at least C++17, attach its
direct dependencies, and install it for `ros2 run`:

```cmake
add_executable(<node_name>_node src/<node_name>_node.cpp)
target_compile_features(<node_name>_node PUBLIC cxx_std_17)
ament_target_dependencies(<node_name>_node rclcpp <message_package>)
install(TARGETS <node_name>_node DESTINATION lib/${PROJECT_NAME})
```

Add a launch file only when it is required to start several nodes or expose
runtime configuration. Do not duplicate launch arguments owned by an included
launch description.

## Validate

Build the target package first:

```bash
scripts/with-ros-env.sh python3 -m colcon build --packages-up-to <package_name>
```

After that build, run a node or inspect topics through the wrapper while
sourcing the overlay:

```bash
scripts/with-ros-env.sh bash -lc 'source install/setup.bash && ros2 run <package_name> <node_name>_node'
scripts/with-ros-env.sh bash -lc 'source install/setup.bash && ros2 topic info -v <topic_name>'
```

Use [add-test-file-cpp](../add-test-file-cpp/) for unit tests and
[launch-testing](../launch-testing/) for end-to-end communication tests. For
ROS-environment escalation or test-log handling, use the linked workspace
build/test skills.

## Related resources

- [create-ros2-package-cpp](../create-ros2-package-cpp/)
- [add-test-file-cpp](../add-test-file-cpp/)
- [launch-testing](../launch-testing/)
- [ros2-workspace-build](../ros2-workspace-build/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
