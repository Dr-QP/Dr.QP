---
name: implement-publisher-subscriber-cpp
description: 'Create ROS 2 C++ publisher and subscriber nodes with rclcpp initialization, message handling, and QoS configuration. Use when implementing topic-based communication in C++, creating rclcpp pub/sub pairs, configuring QoS profiles, or building C++ publisher/subscriber nodes. Keywords: rclcpp, create_publisher, create_subscription, QoS, C++ pub/sub.'
---

# Implement C++ Publisher-Subscriber Pattern

Generate ROS 2 C++ publisher and subscriber nodes with proper rclcpp initialization, message handling, QoS configuration, and project conventions.

For Python nodes, use [implement-publisher-subscriber-python](../implement-publisher-subscriber-python/) instead.

## When to Use This Skill

- Implementing ROS 2 communication between C++ nodes
- Creating rclcpp publisher/subscriber pairs for a message type
- Need QoS configuration for reliable or best-effort communication
- Setting up topic-based communication patterns in C++

## Prerequisites

- Message type defined (built-in or custom)
- Topic name determined
- C++ package exists or will be created
- Understand QoS requirements

## Inputs

- **Message Type**: e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`, `drqp_interfaces/msg/CustomMsg`
- **Topic Name**: e.g., `/my_topic`, `/robot/state`
- **Package Name**: Target package (detect or ask)
- **Node Type**: `publisher`, `subscriber`, or `both`
- **QoS Profile**: `default`, `reliable`, `best_effort`, `sensor_data`, `services`, `parameters`, `system_default`
- **Publish Rate** (for publisher): Hz (default: 10.0)

## Workflow

### Step 1: Validate Inputs

Parse message type (package, message). Validate topic name. Determine if message is built-in or custom (add dependency).

### Step 2: Determine Package and Location

`src/<node_name>_node.cpp`. If package missing, suggest [create-ros2-package-cpp](../create-ros2-package-cpp/) first.

### Step 3: Generate Publisher Node

rclcpp Node, QoS profile, `create_publisher`, timer with `publish_message`. Include copyright, TODO for message population.

### Step 4: Generate Subscriber Node

`create_subscription` with callback. Same QoS as publisher.

### Step 5: Map QoS Profile

| Profile     | Reliability | Durability | History      |
| ----------- | ----------- | ---------- | ------------ |
| default     | Reliable    | Volatile   | Keep last 10 |
| reliable    | Reliable    | Volatile   | Keep last 10 |
| best_effort | Best effort | Volatile   | Keep last 1  |
| sensor_data | Best effort | Volatile   | Keep last 5  |

### Step 6: Update Build Configuration

Add `add_executable`, `ament_target_dependencies`, `install` to CMakeLists.txt.

### Step 7: Update package.xml

Add `rclcpp` and message package dependency.

### Step 8: Launch File (Optional)

Create `launch/<nodes>_launch.py` with Node actions for publisher and subscriber.

## Validation

`colcon build --packages-select <pkg>`, source setup.bash, run both nodes, `ros2 topic echo <topic>`, `ros2 topic info <topic> -v`.

## Edge Cases

- Custom message: Add message package dependency
- QoS mismatch: Warn if publisher/subscriber QoS may be incompatible
- Rate too high: Warn if > 1000 Hz

## Related Resources

- [ROS 2 Writing Publisher/Subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [ROS 2 QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [implement-publisher-subscriber-python](../implement-publisher-subscriber-python/)
- [create-ros2-package-cpp](../create-ros2-package-cpp/)
