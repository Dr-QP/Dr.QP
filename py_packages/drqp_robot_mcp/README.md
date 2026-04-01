# drqp_robot_mcp

Local MCP server for interacting with the Dr.QP Gazebo robot.

This package runs directly inside the current ROS 2 environment. It does not
start Docker, devcontainers, or repository setup scripts.

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

The process expects ROS 2 to already be available in the current shell or
container.

Gazebo support is optional:

- Robot lifecycle and state tools operate against the current ROS graph.
- Gazebo world queries work when Gazebo topics and the `gz` CLI are available.
- Simulation launch is only attempted when the `drqp_gazebo` ROS 2 package is
	installed in the current environment.
