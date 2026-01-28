# drqp_brain Package

This package contains the high-level control logic for the Dr.QP hexapod robot.

## Nodes

### drqp_brain

Main control node that manages robot movement and behavior.

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot movement
- `/joy` (sensor_msgs/Joy): Joystick input for button commands (gait switching, mode changes, etc.)
- `/robot_state` (std_msgs/String): Robot state updates

**Publications:**
- `/joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory): Joint trajectory commands
- `/robot_event` (std_msgs/String): Robot events

**Control Modes:**
- Walk: Uses cmd_vel for locomotion (forward/backward, strafe, rotation)
- BodyPosition: Uses cmd_vel for body translation
- BodyRotation: Uses cmd_vel for body rotation

### joy_to_cmd_vel

Converts joystick input to standard ROS cmd_vel messages.

**Subscriptions:**
- `/joy` (sensor_msgs/Joy): Joystick input

**Publications:**
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands

**Mapping:**
- Left stick Y → linear.x (forward/backward)
- Left stick X → linear.y (left/right strafe)
- Left trigger → linear.z (vertical movement)
- Right stick X → angular.z (rotation)

### drqp_robot_state

Manages robot state machine.

## Architecture

The control architecture follows ROS standards by using the `cmd_vel` topic for velocity commands:

```
Joystick → joy_to_cmd_vel → /cmd_vel → drqp_brain → Joint Trajectories
                  ↓
            Button Commands → drqp_brain (gait, mode switching, etc.)
```

This design:
- Follows ROS industry standards for robot control
- Enables testing without physical joystick hardware
- Allows alternative control sources (autonomous navigation, keyboard, etc.) to send commands via `/cmd_vel`
- Separates movement commands (via `/cmd_vel`) from system commands (via `/joy` buttons)

## Testing

The package can be tested with or without joystick hardware:

**With joystick:**
```bash
ros2 launch drqp_brain bringup.launch.py load_joystick:=true
```

**With cmd_vel commands (programmatic control):**
```bash
# In terminal 1
ros2 launch drqp_brain bringup.launch.py load_joystick:=false

# In terminal 2 - send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

## License

MIT License - see LICENSE file for details.
