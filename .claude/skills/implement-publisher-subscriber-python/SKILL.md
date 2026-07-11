---
name: implement-publisher-subscriber-python
description: 'Create ROS 2 Python publisher and subscriber nodes with rclpy initialization, message handling, and QoS configuration. Use when implementing topic-based communication in Python, creating rclpy pub/sub pairs, configuring QoS profiles, or building Python publisher/subscriber nodes. Keywords: rclpy, create_publisher, create_subscription, QoSProfile, Python pub/sub.'
---

# Implement Python Publisher-Subscriber Pattern

Generate ROS 2 Python publisher and subscriber nodes with proper rclpy initialization, message handling, QoS configuration, and project conventions.

For C++ nodes, use [implement-publisher-subscriber-cpp](../implement-publisher-subscriber-cpp/) instead.

## When to Use This Skill

- Implementing ROS 2 communication between Python nodes
- Creating rclpy publisher/subscriber pairs for a message type
- Need QoS configuration for reliable or best-effort communication
- Setting up topic-based communication patterns in Python

## Prerequisites

- Message type defined (built-in or custom)
- Topic name determined
- Python package exists or will be created
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

`<package>/<node_name>_node.py`. If package missing, suggest [create-ros2-package-python](../create-ros2-package-python/) first.

### Step 3: Generate Publisher Node

rclpy Node, QoSProfile, `create_publisher`, `create_timer`. Include copyright, TODO for message population.

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

Add to `entry_points['console_scripts']` in setup.py.

### Step 7: Update package.xml

Add `rclpy` and message package dependency.

### Step 8: Launch File (Optional)

Create `launch/<nodes>_launch.py` with Node actions for publisher and subscriber.

## Validation

`colcon build --packages-select <pkg>`, source setup.bash, run both nodes, `ros2 topic echo <topic>`, `ros2 topic info <topic> -v`.

## Edge Cases

- Custom message: Add message package dependency
- QoS mismatch: Warn if publisher/subscriber QoS may be incompatible
- Rate too high: Warn if > 1000 Hz

## Related Resources

- [ROS 2 Writing Publisher/Subscriber (Python)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [implement-publisher-subscriber-cpp](../implement-publisher-subscriber-cpp/)
- [create-ros2-package-python](../create-ros2-package-python/)
