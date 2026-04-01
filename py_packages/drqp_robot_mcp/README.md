# drqp_robot_mcp

Local MCP server for interacting with the Dr.QP Gazebo robot.

## Features

- Boot the Gazebo robot to `torque_on`
- Shut the robot down through its lifecycle state machine
- Query the latest robot state, joint states, and robot pose
- Record robot state snapshots over time
- Query Gazebo world entity poses

## Installation

From the repository root:

```bash
uv sync
```

## Running the server

```bash
./.venv/bin/drqp_robot_mcp
```

The server uses Docker Compose to bring up the existing devcontainer service and
then shells into that container for ROS 2 and Gazebo operations.
