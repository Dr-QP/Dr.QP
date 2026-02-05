---
name: ros2-lifecycle-management
description: Manage ROS 2 lifecycle nodes through state transitions. Use when asked to manage lifecycle nodes, configure node states, activate or deactivate nodes, handle lifecycle transitions, recover from failures, understand state machine behavior, or implement lifecycle node patterns. Supports configure, activate, deactivate, cleanup, and shutdown transitions.

---

# ROS 2 Lifecycle Management

Manage ROS 2 lifecycle nodes for controlled initialization, activation, and graceful shutdown with deterministic state transitions.

## When to Use This Skill

- Manage lifecycle node state transitions
- Configure nodes before activation
- Activate nodes for operation
- Deactivate nodes without destroying resources
- Cleanup and shutdown nodes properly
- Handle lifecycle state transition failures
- Implement lifecycle node patterns in code
- Debug lifecycle state issues
- Coordinate startup of multiple lifecycle nodes

## Prerequisites

- ROS 2 Jazzy installation
- Understanding of lifecycle node concept
- Running lifecycle node to manage
- `ros2 lifecycle` command available
- Knowledge of node's configuration requirements

## Lifecycle States

| State | Description | Allowed Transitions |
|-------|-------------|-------------------|
| **Unconfigured** | Initial state, no resources allocated | → Configuring |
| **Inactive** | Configured but not active | → Activating, → CleaningUp |
| **Active** | Fully operational | → Deactivating |
| **Finalized** | Shutdown, resources released | None (terminal state) |

## Lifecycle Transitions

| Transition | From State | To State | Purpose |
|------------|-----------|----------|---------|
| **configure** | Unconfigured | Inactive | Allocate resources, load configuration |
| **activate** | Inactive | Active | Start operation, begin publishing |
| **deactivate** | Active | Inactive | Pause operation, stop publishing |
| **cleanup** | Inactive | Unconfigured | Release resources |
| **shutdown** | Any | Finalized | Emergency shutdown |

## State Machine Diagram

```
         ┌─────────────┐
    ┌───▶│Unconfigured │◀──────┐
    │    └──────┬──────┘       │
    │           │configure     │cleanup
    │           ▼              │
    │    ┌─────────────┐       │
    │    │  Inactive   │───────┘
    │    └──────┬──────┘
    │           │activate
    │           ▼
    │    ┌─────────────┐
    │    │   Active    │
    │    └──────┬──────┘
    │           │deactivate
    │           │
    │           └───────────────┐
    │                           │
    │           shutdown        │
    │      (from any state)     │
    │           │               │
    └───────────▼───────────────┘
         ┌─────────────┐
         │  Finalized  │
         └─────────────┘
```

## Step-by-Step Workflows

### Workflow 1: Basic Lifecycle Node Management

Start and manage a single lifecycle node.

1. Launch a lifecycle node:
   ```bash
   ros2 run <package_name> <lifecycle_node_executable>
   ```

2. Check node's current state:
   ```bash
   ros2 lifecycle get /<node_name>
   ```

3. Configure the node:
   ```bash
   ros2 lifecycle set /<node_name> configure
   ```
   
   The node allocates resources and loads configuration.

4. Verify state changed to Inactive:
   ```bash
   ros2 lifecycle get /<node_name>
   ```

5. Activate the node:
   ```bash
   ros2 lifecycle set /<node_name> activate
   ```
   
   The node starts normal operation.

6. Verify state changed to Active:
   ```bash
   ros2 lifecycle get /<node_name>
   ```

7. To pause operation, deactivate:
   ```bash
   ros2 lifecycle set /<node_name> deactivate
   ```

8. To fully shutdown, cleanup then shutdown:
   ```bash
   ros2 lifecycle set /<node_name> cleanup
   ros2 lifecycle set /<node_name> shutdown
   ```

**When to use**: Basic lifecycle node control, manual state management

### Workflow 2: List and Inspect Lifecycle Nodes

Discover and examine available lifecycle nodes.

1. List all lifecycle nodes:
   ```bash
   ros2 lifecycle nodes
   ```

2. Get detailed node information:
   ```bash
   ros2 node info /<lifecycle_node_name>
   ```

3. Check available states:
   ```bash
   ros2 lifecycle list /<node_name>
   ```

4. Check available transitions from current state:
   ```bash
   ros2 lifecycle list /<node_name> -a
   ```

**When to use**: Discovering lifecycle nodes, understanding available transitions

### Workflow 3: Coordinate Multiple Lifecycle Nodes

Manage startup sequence for system with multiple lifecycle nodes.

1. Launch all lifecycle nodes (in separate terminals or via launch file)

2. List all lifecycle nodes:
   ```bash
   ros2 lifecycle nodes
   ```

3. Configure all nodes in order (dependencies first):
   ```bash
   ros2 lifecycle set /driver_node configure
   ros2 lifecycle set /control_node configure
   ros2 lifecycle set /planning_node configure
   ```

4. Verify all nodes configured:
   ```bash
   for node in driver_node control_node planning_node; do
     echo "$node: $(ros2 lifecycle get /$node)"
   done
   ```

5. Activate nodes in sequence:
   ```bash
   ros2 lifecycle set /driver_node activate
   ros2 lifecycle set /control_node activate
   ros2 lifecycle set /planning_node activate
   ```

6. Verify all nodes active:
   ```bash
   ros2 lifecycle nodes | while read node; do
     echo "$node: $(ros2 lifecycle get $node)"
   done
   ```

**When to use**: Multi-node systems with startup dependencies

### Workflow 4: Handle Lifecycle Failures and Recovery

Recover from failed state transitions.

1. Attempt a transition:
   ```bash
   ros2 lifecycle set /<node_name> configure
   ```

2. If transition fails, node returns to previous state. Check error in node logs:
   ```bash
   ros2 topic echo /rosout | grep <node_name>
   ```

3. Fix the underlying issue (e.g., missing configuration file, hardware not available)

4. Retry the transition:
   ```bash
   ros2 lifecycle set /<node_name> configure
   ```

5. If node stuck in error state, shutdown and restart:
   ```bash
   ros2 lifecycle set /<node_name> shutdown
   # Restart node
   ros2 run <package> <node>
   ```

6. For emergency shutdown from any state:
   ```bash
   ros2 lifecycle set /<node_name> shutdown
   ```

**When to use**: Transition failures, error recovery, emergency stops

### Workflow 5: Implement Lifecycle Node in C++

Create a custom lifecycle node in your code.

**Basic C++ Lifecycle Node:**
```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MyLifecycleNode() : LifecycleNode("my_lifecycle_node")
  {
    RCLCPP_INFO(get_logger(), "Lifecycle node created");
  }

  // Configure transition (Unconfigured → Inactive)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Configuring...");
    
    // Allocate resources, load parameters
    robot_name_ = this->get_parameter("robot_name").as_string();
    
    // Initialize but don't start publishing
    pub_ = this->create_publisher<std_msgs::msg::String>("output", 10);
    
    RCLCPP_INFO(get_logger(), "Successfully configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Activate transition (Inactive → Active)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Activating...");
    
    // Activate publisher
    pub_->on_activate();
    
    // Start timer for publishing
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MyLifecycleNode::publish_message, this));
    
    RCLCPP_INFO(get_logger(), "Successfully activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Deactivate transition (Active → Inactive)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    
    // Stop timer
    timer_->cancel();
    
    // Deactivate publisher
    pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "Successfully deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Cleanup transition (Inactive → Unconfigured)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    
    // Release resources
    timer_.reset();
    pub_.reset();
    
    RCLCPP_INFO(get_logger(), "Successfully cleaned up");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // Shutdown transition (Any → Finalized)
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    
    // Emergency cleanup
    timer_.reset();
    pub_.reset();
    
    RCLCPP_INFO(get_logger(), "Successfully shut down");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  void publish_message()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello from " + robot_name_;
    pub_->publish(msg);
  }

  std::string robot_name_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

**When to use**: Implementing hardware drivers, critical nodes requiring controlled startup

### Workflow 6: Automate Lifecycle Management in Launch Files

Use launch files to automatically manage lifecycle transitions.

```python
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Create lifecycle node
    lifecycle_node = LifecycleNode(
        package='<package_name>',
        executable='<lifecycle_executable>',
        name='<node_name>',
        namespace='',
        output='screen'
    )

    # Configure transition
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == lifecycle_node,
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # When node reaches Inactive, activate it
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == lifecycle_node,
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        lifecycle_node,
        configure_event,
        activate_event
    ])
```

**When to use**: Automated system startup, production deployments

## Lifecycle Best Practices

| Practice | Rationale | Implementation |
|----------|-----------|----------------|
| **Fast transitions** | Avoid blocking in callbacks | Use async operations, timeout checks |
| **Idempotent transitions** | Safe to call multiple times | Check state before acting |
| **Proper error handling** | Return FAILURE on errors | Validate resources before SUCCESS |
| **Resource cleanup** | Prevent leaks | Release in cleanup/shutdown |
| **State logging** | Debug transition issues | Log entry/exit of each callback |
| **Graceful degradation** | Handle partial failures | Don't crash on single component failure |

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Lifecycle node not found" | Node not running or not lifecycle | Verify with `ros2 lifecycle nodes` |
| Transition hangs | Callback blocking or deadlock | Add timeout, check for blocking operations |
| Configure fails | Missing config file or resources | Check node logs, verify configuration |
| Activate fails | Resources not ready | Ensure configure succeeded, check hardware |
| Cannot deactivate | Cleanup code has errors | Check deactivate callback implementation |
| Shutdown incomplete | Resources not released | Verify cleanup/shutdown callbacks |
| Node stuck in transitioning | Callback didn't return | Check for infinite loops or blocking calls |
| Multiple configure calls fail | State not reset properly | Implement proper cleanup in on_cleanup |

## Common Lifecycle Patterns

| Pattern | Use Case | States Used |
|---------|----------|-------------|
| **Simple on/off** | Basic control | Unconfigured ↔ Inactive ↔ Active |
| **Hot standby** | Quick resume | Inactive ↔ Active (stay configured) |
| **Reconfiguration** | Settings change | Active → Inactive → Unconfigured → Inactive → Active |
| **Emergency stop** | Safety shutdown | Any → Finalized (shutdown) |
| **Startup sequence** | Multi-node coordination | Sequential configure → activate |

## Lifecycle vs Regular Nodes

| Aspect | Regular Node | Lifecycle Node |
|--------|-------------|----------------|
| **Startup** | Immediate | Controlled via states |
| **Resource allocation** | In constructor | In on_configure |
| **Operation start** | Immediate | After activate transition |
| **Publisher behavior** | Always active | Active only in Active state |
| **Shutdown** | Destructor | on_cleanup/on_shutdown |
| **Use case** | Simple nodes | Hardware, critical systems |

## When to Use Lifecycle Nodes

**Use lifecycle nodes for:**
- Hardware drivers that need initialization
- Nodes with expensive resource allocation
- Systems requiring coordinated startup
- Nodes that need graceful degradation
- Safety-critical components

**Use regular nodes for:**
- Simple processing nodes
- Stateless transformations
- Visualization tools
- Logging and monitoring

## References

- [ROS 2 Lifecycle Design](https://design.ros2.org/articles/node_lifecycle.html)
- [Managed Nodes Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html#managed-nodes)
- [lifecycle_msgs Package](https://github.com/ros2/rcl_interfaces/tree/jazzy/lifecycle_msgs)
- [rclcpp_lifecycle API](https://docs.ros2.org/jazzy/api/rclcpp_lifecycle/)
