"""Slow end-to-end walking test for the Dr.QP robot MCP controller."""

from __future__ import annotations

import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import pytest

from drqp_robot_mcp.controller import RobotMcpController


def _simulation_available() -> bool:
    if os.environ.get('ROS_DISTRO') is None:
        return False
    if shutil.which('ros2') is None:
        return False
    if subprocess.run(
        ['ros2', 'pkg', 'prefix', 'drqp_gazebo'],
        check=False,
        capture_output=True,
        text=True,
    ).returncode != 0:
        return False
    return True


def _pose_distance(before, after) -> float:
    delta_x = after.position.x - before.position.x
    delta_y = after.position.y - before.position.y
    delta_z = after.position.z - before.position.z
    return math.sqrt((delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z))


def _terminate_simulation(controller: RobotMcpController) -> None:
    pid_text = controller.launch_pid_path.read_text(encoding='utf-8').strip()
    pid = int(pid_text)
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return


@pytest.mark.slow
@pytest.mark.skipif(not _simulation_available(), reason='ROS 2 Gazebo environment is unavailable.')
def test_walk_for_duration_moves_robot_in_simulation() -> None:
    """The higher-level walk command moves the robot in Gazebo end to end."""
    controller = RobotMcpController(workspace_root=Path(__file__).resolve().parents[3])
    boot_result = controller.boot_up(timeout_sec=120.0)

    try:
        start_state = controller.get_robot_state(timeout_sec=10.0)
        assert start_state.robot_pose is not None

        sequence_result = controller.walk_for_duration(
            stride_x=1.0,
            duration_sec=2.0,
            publish_hz=5.0,
        )
        assert sequence_result.publish_count == 10

        deadline = time.time() + 15.0
        moved_distance = 0.0
        end_state = start_state
        while time.time() < deadline:
            end_state = controller.get_robot_state(timeout_sec=10.0)
            if end_state.robot_pose is None:
                time.sleep(0.5)
                continue
            moved_distance = _pose_distance(start_state.robot_pose, end_state.robot_pose)
            if moved_distance >= 0.02:
                break
            time.sleep(0.5)

        assert end_state.robot_pose is not None
        assert moved_distance >= 0.02
    finally:
        controller.stop_motion()
        controller.shut_down(timeout_sec=60.0)
        if boot_result.simulation_was_started and controller.launch_pid_path.exists():
            _terminate_simulation(controller)