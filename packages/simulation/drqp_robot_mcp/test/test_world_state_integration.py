"""Integration tests for Gazebo Transport world state in the MCP runtime."""

from __future__ import annotations

import os
import signal
import subprocess
import time

from drqp_robot_mcp import runtime
from drqp_robot_mcp.controller import RobotMcpController
import pytest


def _terminate_simulation(controller: RobotMcpController) -> None:
    pid_text = controller.launch_pid_path.read_text(encoding='utf-8').strip()
    pid = int(pid_text)
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def _reset_local_simulation_state() -> None:
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


@pytest.mark.slow
def test_get_world_state_uses_live_gazebo_transport() -> None:
    """World snapshots should come from Gazebo Transport and keep updating."""
    _reset_local_simulation_state()
    controller = RobotMcpController()
    simulation = runtime.start_simulation(
        pid_path=controller.launch_pid_path,
        log_path=controller.launch_log_path,
    )
    simulation_was_started = bool(simulation.get('started', False))

    try:
        first_snapshot = controller.get_world_state(timeout_sec=10.0)

        assert first_snapshot.available is True
        assert first_snapshot.source == 'gazebo'
        assert first_snapshot.world_name == controller.world_name
        assert first_snapshot.simulation_time_sec is not None
        assert first_snapshot.entity_count > 0

        entity_names = {entity.name for entity in first_snapshot.entities}
        assert 'drqp' in entity_names

        drqp_entity = next(entity for entity in first_snapshot.entities if entity.name == 'drqp')
        assert drqp_entity.entity_id is not None
        assert drqp_entity.pose is not None

        deadline = time.monotonic() + 10.0
        updated_snapshot = None
        while time.monotonic() < deadline:
            time.sleep(0.5)
            candidate = controller.get_world_state(timeout_sec=2.0)
            if (
                candidate.simulation_time_sec is not None
                and first_snapshot.simulation_time_sec is not None
                and candidate.simulation_time_sec > first_snapshot.simulation_time_sec
            ):
                updated_snapshot = candidate
                break

        assert updated_snapshot is not None
        assert updated_snapshot.available is True
        assert updated_snapshot.source == 'gazebo'
        assert updated_snapshot.entity_count >= first_snapshot.entity_count
    finally:
        controller.close()
        runtime.shutdown_default_ros_runtime()
        if simulation_was_started and controller.launch_pid_path.exists():
            _terminate_simulation(controller)
