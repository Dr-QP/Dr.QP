---
name: implement-publisher-subscriber-python
description: Implement rclpy publisher, subscriber, or paired nodes with complete executable shutdown, compatible QoS, package dependencies, and targeted validation. Use when adding Python ROS 2 topic communication or configuring an rclpy QoS profile.
---

# Implement Python Publisher-Subscriber Pattern

Implement a small, observable rclpy pub/sub node or pair. For C++ nodes, use
[implement-publisher-subscriber-cpp](../implement-publisher-subscriber-cpp/).

## Gather compatible inputs

Require the message type, absolute or relative topic name, target package, node
role (`publisher`, `subscriber`, or `both`), and a publish rate when needed.
Accept only these QoS selections: `default`, `reliable`, `best_effort`, and
`sensor_data`. Reject unsupported names rather than guessing service,
parameter, or system-default semantics.

Declare `rclpy` and the message package in `package.xml`; add custom message
packages as explicit dependencies. Put each node module inside the package's
inner Python package and add its console script in `setup.py` only when it is
an executable.

## Map QoS deliberately

Use the same QoS for both endpoints unless a documented compatibility reason
requires otherwise. `sensor_data` should use the rclpy supplied profile; the
other choices should state their policies explicitly:

| Input         | `QoSProfile` shape                       |
| ------------- | ---------------------------------------- |
| `default`     | `QoSProfile(depth=10)`                   |
| `reliable`    | depth 10, `ReliabilityPolicy.RELIABLE`   |
| `best_effort` | depth 1, `ReliabilityPolicy.BEST_EFFORT` |
| `sensor_data` | `qos_profile_sensor_data`                |

Do not promise delivery when a best-effort publisher meets a reliable
subscriber: requested reliability must be compatible with what the publisher
offers. Use a latched/transient-local profile only when the topic contract
requires late joiners to receive the last value, and document that choice.

## Implement nodes and the executable lifecycle

Create publishers/subscriptions in the node constructor, retain the returned
handles, validate configured rates, and keep callbacks non-blocking. Use a
timer for periodic publishing. Provide a complete `main()` around each console
entry point:

```python
def main() -> int:
    node = None
    try:
        rclpy.init()
        node = ExampleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
```

Import `ExternalShutdownException` from `rclpy.executors`; allow unexpected
exceptions to surface after cleanup. An optional launch file belongs in
`launch/` and needs the corresponding `launch`/`launch_ros` dependencies.

## Validate the package

Add unit or node tests with [add-test-file-python](../add-test-file-python/),
then build and test only the affected package:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --packages-up-to <package_name> --symlink-install
scripts/with-ros-env.sh python3 -m colcon test \
  --packages-select <package_name> --mixin coverage-pytest \
  --return-code-on-test-failure
```

Use [ros2-workspace-testing](../ros2-workspace-testing/) for result logs and
the escalation path; use [ros2-diagnostics](../ros2-diagnostics/) to inspect a
running topic without turning this implementation skill into a diagnostics
workflow.

## Related resources

- [ROS 2 Python pub/sub tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 QoS concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [create-ros2-package-python](../create-ros2-package-python/)
