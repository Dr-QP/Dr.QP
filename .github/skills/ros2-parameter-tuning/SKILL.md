---
name: ros2-parameter-tuning
description: Configure and tune ROS 2 node parameters using YAML files and ros2 param commands. Use when asked to configure parameters, create parameter files, update parameters at runtime, validate parameter syntax, manage parameter namespaces, tune node behavior, or troubleshoot parameter loading issues. Supports static configuration files and dynamic parameter updates.

---

# ROS 2 Parameter Tuning

Configure and dynamically tune ROS 2 node parameters using YAML configuration files and runtime parameter commands.

## When to Use This Skill

- Create parameter YAML files for node configuration
- Update parameters at runtime without restarting nodes
- Validate parameter file syntax
- Manage parameter namespaces
- Tune control parameters for optimal performance
- Debug parameter loading issues
- Save runtime parameters to file
- Set parameter defaults in code

## Prerequisites

- ROS 2 Jazzy installation
- Running ROS 2 nodes to configure
- Understanding of node parameter requirements
- `ros2 param` command available
- Text editor for YAML files

## Parameter Types

| Type | Example Value | Python Type | YAML Syntax |
|------|--------------|-------------|-------------|
| **bool** | `true`, `false` | `bool` | `param: true` |
| **int** | `42`, `-10` | `int` | `param: 42` |
| **double** | `3.14`, `-2.5` | `float` | `param: 3.14` |
| **string** | `"hello"` | `str` | `param: "hello"` |
| **int_array** | `[1, 2, 3]` | `List[int]` | `param: [1, 2, 3]` |
| **double_array** | `[1.0, 2.5]` | `List[float]` | `param: [1.0, 2.5]` |
| **string_array** | `["a", "b"]` | `List[str]` | `param: ["a", "b"]` |

## Step-by-Step Workflows

### Workflow 1: Create Parameter YAML File

Create a parameter configuration file for node initialization.

1. Create config directory in package:
   ```bash
   mkdir -p <workspace_root>/packages/runtime/<package_name>/config
   ```

2. Create parameter file (e.g., `config/robot_params.yaml`):
   ```yaml
   # Node name with double wildcard for namespace flexibility
   /**:
     ros__parameters:
       # Robot identification
       robot_name: "Dr.QP"
       robot_id: 1
       
       # Control parameters
       control_frequency: 50.0
       max_velocity: 1.5
       max_acceleration: 0.5
       
       # Safety limits
       enable_safety: true
       emergency_stop_enabled: true
       
       # Communication settings
       serial_port: "/dev/ttyUSB0"
       baudrate: 115200
       timeout_ms: 1000
       
       # Arrays
       joint_names: ["joint1", "joint2", "joint3", "joint4"]
       position_limits: [0.0, 180.0, 0.0, 180.0]
   ```

3. Install config directory in `CMakeLists.txt`:
   ```cmake
   install(DIRECTORY config
     DESTINATION share/${PROJECT_NAME}
   )
   ```

4. Rebuild package:
   ```bash
   colcon build --packages-select <package_name> --symlink-install
   ```

5. Load parameters in launch file:
   ```python
   import os
   from ament_index_python.packages import get_package_share_directory
   
   params_file = os.path.join(
       get_package_share_directory('<package_name>'),
       'config',
       'robot_params.yaml'
   )
   
   node = Node(
       package='<package_name>',
       executable='<node_name>',
       parameters=[params_file]
   )
   ```

**When to use**: Initial node configuration, environment-specific settings

### Workflow 2: View and List Node Parameters

Inspect current parameter values of running nodes.

1. List all running nodes:
   ```bash
   ros2 node list
   ```

2. List all parameters for a specific node:
   ```bash
   ros2 param list /<node_name>
   ```

3. Get value of a specific parameter:
   ```bash
   ros2 param get /<node_name> <parameter_name>
   ```

4. Get all parameters in YAML format:
   ```bash
   ros2 param dump /<node_name>
   ```

5. Save parameters to file:
   ```bash
   ros2 param dump /<node_name> --output-dir ./config/
   ```

**When to use**: Inspecting runtime configuration, debugging parameter issues

### Workflow 3: Update Parameters at Runtime

Change node parameters dynamically without restarting.

1. Set a single parameter:
   ```bash
   ros2 param set /<node_name> <parameter_name> <value>
   ```

   Examples:
   ```bash
   ros2 param set /controller max_velocity 2.0
   ros2 param set /serial_driver baudrate 115200
   ros2 param set /robot enable_safety true
   ```

2. Verify parameter was updated:
   ```bash
   ros2 param get /<node_name> <parameter_name>
   ```

3. Load multiple parameters from file:
   ```bash
   ros2 param load /<node_name> <params_file>.yaml
   ```

4. Monitor parameter changes in real-time (if node publishes them):
   ```bash
   ros2 topic echo /parameter_events
   ```

**Important**: Not all parameters support runtime updates. Some nodes require restart for changes to take effect.

**When to use**: Tuning control parameters, testing different configurations

### Workflow 4: Parameter Namespace Management

Organize parameters using namespaces for multi-robot systems.

1. Create namespaced parameter file (`config/robot1_params.yaml`):
   ```yaml
   /robot1/**:
     ros__parameters:
       robot_name: "robot1"
       max_velocity: 1.5
       
   /robot2/**:
     ros__parameters:
       robot_name: "robot2"
       max_velocity: 2.0
   ```

2. Launch nodes with namespaces:
   ```python
   robot1_node = Node(
       package='<package>',
       executable='<node>',
       name='controller',
       namespace='robot1',
       parameters=[params_file]
   )
   
   robot2_node = Node(
       package='<package>',
       executable='<node>',
       name='controller',
       namespace='robot2',
       parameters=[params_file]
   )
   ```

3. Access namespaced parameters:
   ```bash
   ros2 param get /robot1/controller max_velocity
   ros2 param get /robot2/controller max_velocity
   ```

4. Update namespaced parameters:
   ```bash
   ros2 param set /robot1/controller max_velocity 1.8
   ```

**When to use**: Multi-robot systems, component reuse with different configs

### Workflow 5: Validate Parameter File Syntax

Check parameter files for syntax errors before using them.

1. Attempt to parse YAML file with Python:
   ```bash
   python3 -c "import yaml; yaml.safe_load(open('config/params.yaml'))"
   ```

2. Check for common YAML errors:
   - Consistent indentation (use spaces, not tabs)
   - Proper quoting of strings with special characters
   - Correct list syntax `[item1, item2]` or multi-line
   - Boolean values: `true`/`false` (lowercase)

3. Test parameter file by loading into a running node:
   ```bash
   ros2 param load /<node_name> config/params.yaml
   ```

4. Check for parameter type mismatches in node logs:
   ```bash
   ros2 run <package> <executable> --ros-args --log-level debug
   ```

**When to use**: Before committing parameter files, debugging load issues

### Workflow 6: Declare and Use Parameters in Node Code

Define parameters in your ROS 2 node implementation.

**C++ Example:**
```cpp
class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Declare parameters with defaults
    this->declare_parameter<std::string>("robot_name", "default_robot");
    this->declare_parameter<double>("max_velocity", 1.0);
    this->declare_parameter<int>("control_frequency", 50);
    this->declare_parameter<bool>("enable_safety", true);
    
    // Get parameter values
    robot_name_ = this->get_parameter("robot_name").as_string();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_int();
    enable_safety_ = this->get_parameter("enable_safety").as_bool();
    
    RCLCPP_INFO(this->get_logger(), "Robot name: %s", robot_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Max velocity: %.2f", max_velocity_);
  }

private:
  std::string robot_name_;
  double max_velocity_;
  int control_frequency_;
  bool enable_safety_;
};
```

**Python Example:**
```python
from rclcpp.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameters with defaults
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('enable_safety', True)
        
        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.enable_safety = self.get_parameter('enable_safety').value
        
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
```

**When to use**: Implementing new nodes, defining node configuration interface

## Parameter Best Practices

| Practice | Rationale | Implementation |
|----------|-----------|----------------|
| **Declare with defaults** | Ensures node works without config file | Use `declare_parameter()` with default value |
| **Validate ranges** | Prevents invalid configurations | Check parameter values after loading |
| **Use descriptive names** | Makes configuration self-documenting | `max_velocity` not `mv` |
| **Group related params** | Improves organization | Use common prefixes or namespaces |
| **Document parameters** | Helps users configure correctly | Add comments in YAML, README |
| **Version config files** | Tracks configuration changes | Commit YAML files to git |

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Parameter not declared" | Node doesn't recognize parameter | Check parameter name spelling, verify node declares it |
| Parameters not loaded | YAML syntax error or wrong file path | Validate YAML syntax, check file exists in install directory |
| Wrong parameter type | Type mismatch (int vs double, etc.) | Check YAML value matches declared type |
| Parameters reset after launch | Not persisted or not loaded from file | Use `parameters=[params_file]` in launch file |
| Namespace issues | Wrong namespace in YAML | Use `/**:` wildcard or correct namespace `/robot1/**:` |
| Runtime updates don't work | Node doesn't support dynamic parameters | Some parameters require node restart |
| "No such file or directory" | Config file not installed | Add `install(DIRECTORY config ...)` to CMakeLists.txt |
| Boolean not working | Used TRUE/FALSE instead of lowercase | Use lowercase `true`/`false` in YAML |

## Common Parameter Patterns

| Pattern | YAML Example | Use Case |
|---------|-------------|----------|
| **Basic values** | `param: 42` | Simple configuration |
| **Namespaced** | `/ns/**:\n  ros__parameters:` | Multi-robot systems |
| **Arrays** | `joints: [1, 2, 3]` | Multiple values |
| **Nested structure** | Use separate params | Complex configuration |
| **Environment-specific** | Multiple YAML files | dev/staging/prod configs |

## Parameter File Locations

Recommended structure:
```
<package_name>/
├── config/
│   ├── default_params.yaml      # Default configuration
│   ├── robot1_params.yaml       # Robot-specific config
│   ├── simulation_params.yaml   # Simulation settings
│   └── production_params.yaml   # Production settings
├── launch/
│   └── robot.launch.py          # Loads appropriate params
└── CMakeLists.txt               # Installs config/
```

## References

- [ROS 2 Parameters Guide](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Using Parameters in ROS 2 Nodes](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
- [Parameter YAML File Format](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros2-param.html)
