"""Unit tests for devcontainer-side Gazebo parsing helpers."""

from __future__ import annotations

from drqp_robot_mcp.container_cli import parse_gazebo_pose_info


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

    parsed = parse_gazebo_pose_info(raw_output)

    assert len(parsed) == 2
    assert parsed[1]['name'] == 'drqp'
    assert parsed[1]['entity_id'] == 2
    assert parsed[1]['pose']['position']['x'] == 1.5
    assert parsed[1]['pose']['orientation']['w'] == 0.707
