"""Shared test helpers for drqp_robot_mcp integration tests."""

from __future__ import annotations

import os
import signal
import subprocess

from drqp_robot_mcp import runtime
from drqp_robot_mcp.controller import RobotMcpController


def reset_local_simulation_state() -> None:
    """Clear any stale local runtime or Gazebo processes before booting."""
    runtime.shutdown_default_ros_runtime()
    subprocess.run(
        ['pkill', '-9', '-f', 'ros2 launch drqp_gazebo sim.launch.py'],
        check=False,
    )
    subprocess.run(
        ['pkill', '-9', '-f', 'ruby .*/gz sim -r -v 3 empty.sdf'],
        check=False,
    )
    subprocess.run(
        ['pkill', '-9', '-f', '^gz sim -r -v 3 empty.sdf'],
        check=False,
    )


def terminate_simulation(controller: RobotMcpController) -> None:
    """Send SIGTERM to the Gazebo process group for the given controller."""
    pid_text = controller.launch_pid_path.read_text(encoding='utf-8').strip()
    pid = int(pid_text)
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
