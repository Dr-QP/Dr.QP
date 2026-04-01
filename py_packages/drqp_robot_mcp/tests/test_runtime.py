"""Unit tests for local ROS and Gazebo runtime helpers."""

from __future__ import annotations

from argparse import Namespace
from pathlib import Path

from drqp_robot_mcp import runtime


def test_parse_gazebo_pose_info_extracts_entity_poses() -> None:
    """Gazebo protobuf text output is converted into structured entity poses."""
    raw_output = """
header {
  stamp {
    sec: 12
    nsec: 34
  }
}
pose {
  name: "ground_plane"
  id: 1
  position {
    x: 0
    y: 0
    z: 0
  }
  orientation {
    x: 0
    y: 0
    z: 0
    w: 1
  }
}
pose {
  name: "drqp"
  id: 2
  position {
    x: 1.5
    y: -0.25
    z: 0.4
  }
  orientation {
    x: 0
    y: 0
    z: 0.707
    w: 0.707
  }
}
"""

    parsed = runtime.parse_gazebo_pose_info(raw_output)

    assert len(parsed) == 2
    assert parsed[1]['name'] == 'drqp'
    assert parsed[1]['entity_id'] == 2
    assert parsed[1]['pose']['position']['x'] == 1.5
    assert parsed[1]['pose']['orientation']['w'] == 0.707


def test_start_simulation_returns_unavailable_when_gazebo_launch_is_missing(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Start simulation returns a structured response when Gazebo is absent."""
    pid_path = tmp_path / 'sim.pid'
    log_path = tmp_path / 'sim.log'

    monkeypatch.setattr(runtime, '_gazebo_launch_is_available', lambda: False)

    result = runtime.start_simulation(
        workspace_root=tmp_path,
        pid_path=pid_path,
        log_path=log_path,
    )

    assert result['started'] is False
    assert result['available'] is False
    assert 'Gazebo launch is not available' in result['message']


def test_cmd_start_simulation_uses_direct_ros2_launch_without_shell(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Simulation launch uses a direct local argv command."""
    captured: dict[str, object] = {}
    pid_path = tmp_path / 'sim.pid'
    log_path = tmp_path / 'sim.log'

    class FakeProcess:
        pid = 4321

    def fake_popen(command, **kwargs):
        captured['command'] = command
        captured['kwargs'] = kwargs
        return FakeProcess()

    monkeypatch.setattr(runtime, '_gazebo_launch_is_available', lambda: True)
    monkeypatch.setattr(runtime.subprocess, 'Popen', fake_popen)

    result = runtime.start_simulation(
        workspace_root=tmp_path,
        pid_path=pid_path,
        log_path=log_path,
    )

    assert result['started'] is True
    assert captured['command'] == [
        'ros2',
        'launch',
        'drqp_gazebo',
        'sim.launch.py',
        'gui:=false',
    ]
    assert captured['kwargs']['cwd'] == tmp_path
    assert captured['kwargs']['start_new_session'] is True
    assert 'shell' not in captured['kwargs']


def test_cmd_start_simulation_namespace_adapter(tmp_path: Path, monkeypatch) -> None:
    """The CLI-style adapter delegates to the local runtime helper."""
    pid_path = tmp_path / 'sim.pid'
    log_path = tmp_path / 'sim.log'

    monkeypatch.setattr(
        runtime,
        'start_simulation',
        lambda workspace_root, pid_path, log_path, gui=False: {
            'started': True,
            'available': True,
            'pid': 99,
            'message': f'{workspace_root}:{pid_path}:{log_path}:{gui}',
        },
    )

    result = runtime.cmd_start_simulation(
        Namespace(
            workspace_root=str(tmp_path),
            pid_path=str(pid_path),
            log_path=str(log_path),
            gui=False,
        )
    )

    assert result['started'] is True
    assert result['available'] is True