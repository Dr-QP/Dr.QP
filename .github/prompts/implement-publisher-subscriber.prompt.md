---
description: 'Create ROS 2 publisher and subscriber nodes with proper initialization, message handling, and QoS configuration'
name: 'implement-publisher-subscriber'
agent: 'agent'
tools: ['read', 'edit', 'search']
argument-hint: '<message-type> <topic-name> [--qos reliable|best-effort]'
---

# Implement Publisher-Subscriber Pattern

Generate ROS 2 publisher and subscriber nodes with proper initialization, message handling, QoS configuration, and following project conventions.

## When to Use This Prompt

- Implementing ROS 2 communication between nodes
- Creating publisher/subscriber pairs for a specific message type
- Need proper QoS configuration for reliable or best-effort communication
- Setting up topic-based communication patterns
- Creating example or template nodes for message passing

## Prerequisites

- Message type is defined (built-in or custom)
- Topic name is determined
- Package exists or will be created
- Understand QoS requirements (reliability, durability, history)

## Inputs

### Required Inputs

- **Message Type** `${input:messageType:std_msgs/msg/String}`: ROS 2 message type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`, `drqp_interfaces/msg/CustomMsg`)
- **Topic Name** `${input:topicName:/my_topic}`: Name of the topic for communication (e.g., `/my_topic`, `/robot/state`)

### Optional Inputs

- **Package Name** `${input:packageName}`: Target package (detect from context or ask)
- **Node Type** `${input:nodeType:both}`: Create `publisher`, `subscriber`, or `both`
- **Language** `${input:language:cpp}`: Implementation language (`cpp` or `python`)
- **QoS Profile** `${input:qos:default}`: QoS profile (`default`, `reliable`, `best_effort`, `sensor_data`, `services`, `parameters`, `system_default`)
- **Publish Rate** (for publisher): Publishing frequency in Hz (default: 10.0)

## Workflow

### Step 1: Validate Inputs

1. Parse message type to extract:
   - Package: `std_msgs`, `sensor_msgs`, `drqp_interfaces`, etc.
   - Message: `String`, `Image`, `CustomMsg`, etc.
2. Validate topic name follows ROS 2 conventions (starts with `/` or is relative)
3. Determine if message type is built-in or custom (requires dependency)
4. Ask for missing required inputs

### Step 2: Determine Package and Location

1. If package name provided, use it
2. Otherwise, check current workspace context or ask user
3. Determine if package exists:
   - If exists: Add nodes to existing package
   - If not: Suggest creating package first (use create-ros2-package prompt)
4. Determine source file locations:
   - C++: `<package>/src/<node_name>_node.cpp`
   - Python: `<package>/<package>/<node_name>_node.py`

### Step 3: Generate Publisher Node

#### For C++ Publisher:

```cpp
// Copyright notice

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <<message_package>/<message_header>.hpp>

using namespace std::chrono_literals;

class <TopicName>Publisher : public rclcpp::Node {
public:
  <TopicName>Publisher()
  : Node("<node_name>_publisher")
  {
    // Configure QoS
    auto qos = <qos_profile>;
    
    // Create publisher
    publisher_ = this->create_publisher<<MessageType>>("<topic_name>", qos);
    
    // Create timer
    timer_ = this->create_wall_timer(
      <publish_period>ms,
      std::bind(&<TopicName>Publisher::publish_message, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Publisher initialized on topic: <topic_name>");
  }

private:
  void publish_message() {
    auto message = <MessageType>();
    
    // Populate message fields
    // TODO: Set message content
    
    publisher_->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "Published message");
  }

  rclcpp::Publisher<<MessageType>>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<<TopicName>Publisher>());
  rclcpp::shutdown();
  return 0;
}
```

#### For Python Publisher:

```python
"""<TopicName> publisher node."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, <QoS_settings>
from <message_package>.msg import <MessageClass>


class <TopicName>Publisher(Node):
    """Publisher node for <topic_name> topic."""

    def __init__(self):
        super().__init__('<node_name>_publisher')
        
        # Configure QoS
        qos_profile = QoSProfile(<qos_settings>)
        
        # Create publisher
        self.publisher_ = self.create_publisher(
            <MessageClass>,
            '<topic_name>',
            qos_profile
        )
        
        # Create timer
        timer_period = <period>  # seconds
        self.timer = self.create_timer(timer_period, self.publish_message)
        
        self.get_logger().info('Publisher initialized on topic: <topic_name>')

    def publish_message(self):
        """Publish message to topic."""
        msg = <MessageClass>()
        
        # TODO: Populate message fields
        
        self.publisher_.publish(msg)
        self.get_logger().debug('Published message')


def main(args=None):
    rclpy.init(args=args)
    node = <TopicName>Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 4: Generate Subscriber Node

#### For C++ Subscriber:

```cpp
// Copyright notice

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <<message_package>/<message_header>.hpp>

class <TopicName>Subscriber : public rclcpp::Node {
public:
  <TopicName>Subscriber()
  : Node("<node_name>_subscriber")
  {
    // Configure QoS
    auto qos = <qos_profile>;
    
    // Create subscription
    subscription_ = this->create_subscription<<MessageType>>(
      "<topic_name>",
      qos,
      std::bind(&<TopicName>Subscriber::message_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Subscriber initialized on topic: <topic_name>");
  }

private:
  void message_callback(const <MessageType>::SharedPtr msg) {
    // Process received message
    RCLCPP_INFO(this->get_logger(), "Received message");
    
    // TODO: Process message content
  }

  rclcpp::Subscription<<MessageType>>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<<TopicName>Subscriber>());
  rclcpp::shutdown();
  return 0;
}
```

#### For Python Subscriber:

```python
"""<TopicName> subscriber node."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, <QoS_settings>
from <message_package>.msg import <MessageClass>


class <TopicName>Subscriber(Node):
    """Subscriber node for <topic_name> topic."""

    def __init__(self):
        super().__init__('<node_name>_subscriber')
        
        # Configure QoS
        qos_profile = QoSProfile(<qos_settings>)
        
        # Create subscription
        self.subscription = self.create_subscription(
            <MessageClass>,
            '<topic_name>',
            self.message_callback,
            qos_profile
        )
        
        self.get_logger().info('Subscriber initialized on topic: <topic_name>')

    def message_callback(self, msg):
        """Handle received messages."""
        self.get_logger().info('Received message')
        
        # TODO: Process message content


def main(args=None):
    rclpy.init(args=args)
    node = <TopicName>Subscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Configure QoS Profile

Map QoS input to appropriate settings:

| QoS Profile | Reliability | Durability | History | Use Case |
|-------------|-------------|------------|---------|----------|
| `default` | Reliable | Volatile | Keep last (10) | General purpose |
| `reliable` | Reliable | Volatile | Keep last (10) | Critical data |
| `best_effort` | Best effort | Volatile | Keep last (1) | High-frequency data |
| `sensor_data` | Best effort | Volatile | Keep last (5) | Sensor streams |
| `services` | Reliable | Volatile | Keep last (10) | Service calls |
| `parameters` | Reliable | Volatile | Keep last (1000) | Parameter updates |
| `system_default` | System default | System default | System default | Use ROS defaults |

Generate appropriate QoS code based on selection.

### Step 6: Update Build Configuration

#### For C++ (Update CMakeLists.txt):

```cmake
# Add executable(s)
add_executable(<node_name>_publisher_node
  src/<node_name>_publisher_node.cpp
)
ament_target_dependencies(<node_name>_publisher_node
  rclcpp
  <message_package>
)

add_executable(<node_name>_subscriber_node
  src/<node_name>_subscriber_node.cpp
)
ament_target_dependencies(<node_name>_subscriber_node
  rclcpp
  <message_package>
)

# Install executables
install(TARGETS
  <node_name>_publisher_node
  <node_name>_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)
```

#### For Python (Update setup.py):

Add to `entry_points['console_scripts']`:
```python
'<node_name>_publisher = <package_name>.<node_name>_publisher_node:main',
'<node_name>_subscriber = <package_name>.<node_name>_subscriber_node:main',
```

### Step 7: Update Package Dependencies

Add message package dependency to package.xml:
```xml
<depend>rclcpp</depend>  <!-- or rclpy for Python -->
<depend><message_package></depend>
```

If message package is not already listed, add it.

### Step 8: Generate Launch File (Optional)

Create `launch/<nodes>_launch.py`:

```python
"""Launch publisher and subscriber nodes."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<node_name>_publisher',
            name='<node_name>_publisher',
            output='screen',
            parameters=[{
                # Add parameters here
            }]
        ),
        Node(
            package='<package_name>',
            executable='<node_name>_subscriber',
            name='<node_name>_subscriber',
            output='screen',
            parameters=[{
                # Add parameters here
            }]
        ),
    ])
```

## Output Expectations

### Success Criteria

- Publisher node created with proper initialization
- Subscriber node created with message callback
- QoS profiles configured appropriately
- Build system updated with new executables
- Package dependencies updated
- Launch file created (optional)
- TODO comments indicate where to add custom logic

### Generated Output Summary

```
✅ Created ROS 2 Publisher-Subscriber nodes
📦 Package: <package_name>
📨 Message type: <message_type>
📡 Topic: <topic_name>
⚙️  QoS: <qos_profile>

📝 Generated files:
   Publisher: <publisher_file_path>
   Subscriber: <subscriber_file_path>
   Launch file: <launch_file_path> (optional)

🔨 Build configuration updated:
   - CMakeLists.txt (C++) or setup.py (Python)
   - package.xml dependencies

🧪 Test the nodes:
   1. Build: colcon build --packages-select <package_name>
   2. Source: source install/setup.bash
   3. Terminal 1: ros2 run <package_name> <node_name>_publisher
   4. Terminal 2: ros2 run <package_name> <node_name>_subscriber
   5. Or use launch: ros2 launch <package_name> <nodes>_launch.py

📊 Verify communication:
   ros2 topic echo <topic_name>
   ros2 topic info <topic_name>
```

## Validation Steps

1. **Build package**: `colcon build --packages-select <package_name>`
2. **Source workspace**: `source install/setup.bash`
3. **Run publisher**: `ros2 run <package_name> <node_name>_publisher`
4. **Run subscriber**: `ros2 run <package_name> <node_name>_subscriber` (separate terminal)
5. **Check topics**: `ros2 topic list`
6. **Echo messages**: `ros2 topic echo <topic_name>`
7. **Verify QoS**: `ros2 topic info <topic_name> -v`

## Quality Assurance

- [ ] Publisher initializes and publishes at specified rate
- [ ] Subscriber receives and processes messages
- [ ] QoS profiles are correctly configured
- [ ] Message types are correctly imported/included
- [ ] Build system includes new executables
- [ ] Package dependencies are complete
- [ ] Nodes log initialization and activity
- [ ] Proper cleanup in destructors/shutdown
- [ ] TODO comments guide user customization
- [ ] Launch file runs both nodes successfully

## Edge Cases

- **Custom message type**: Ensure message package dependency is added
- **Invalid topic name**: Validate and suggest corrections
- **QoS mismatch**: Warn if publisher/subscriber QoS might not be compatible
- **Missing message package**: Detect and report if message package doesn't exist
- **Namespace conflicts**: Ensure node names don't conflict with existing nodes
- **Rate too high**: Warn if publish rate is unreasonably high (> 1000 Hz)

## Related Resources

- [ROS 2 Writing Publisher/Subscriber](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [ROS 2 QoS Documentation](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [Engineering guidelines](/.github/instructions/engineering.instructions.md)
- [create-ros2-package prompt](./create-ros2-package.prompt.md)
