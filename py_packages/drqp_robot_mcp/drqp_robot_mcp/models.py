"""Structured models returned by the Dr.QP robot MCP tools."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class Vector3:
    """Three-dimensional vector."""

    x: float
    y: float
    z: float

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> 'Vector3 | None':
        """Create a vector from a JSON mapping."""
        if data is None:
            return None
        return cls(
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            z=float(data.get('z', 0.0)),
        )


@dataclass
class Quaternion:
    """Quaternion orientation."""

    x: float
    y: float
    z: float
    w: float

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> 'Quaternion | None':
        """Create an orientation from a JSON mapping."""
        if data is None:
            return None
        return cls(
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            z=float(data.get('z', 0.0)),
            w=float(data.get('w', 1.0)),
        )


@dataclass
class Pose:
    """Entity pose in the world."""

    position: Vector3
    orientation: Quaternion

    @classmethod
    def from_mapping(cls, data: dict[str, Any] | None) -> 'Pose | None':
        """Create a pose from a JSON mapping."""
        if data is None:
            return None
        position = Vector3.from_mapping(data.get('position'))
        orientation = Quaternion.from_mapping(data.get('orientation'))
        if position is None or orientation is None:
            return None
        return cls(position=position, orientation=orientation)


@dataclass
class JointStateValue:
    """Joint state snapshot."""

    position: float | None
    velocity: float | None
    effort: float | None

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> 'JointStateValue':
        """Create a joint state from a JSON mapping."""
        return cls(
            position=_optional_float(data.get('position')),
            velocity=_optional_float(data.get('velocity')),
            effort=_optional_float(data.get('effort')),
        )


@dataclass
class WorldEntityState:
    """Gazebo world entity pose snapshot."""

    name: str
    entity_id: int | None
    pose: Pose | None

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> 'WorldEntityState':
        """Create an entity state from a JSON mapping."""
        return cls(
            name=str(data.get('name', '')),
            entity_id=_optional_int(data.get('entity_id')),
            pose=Pose.from_mapping(data.get('pose')),
        )


@dataclass
class WorldStateSnapshot:
    """Gazebo world snapshot."""

    available: bool
    world_name: str
    simulation_time_sec: float | None
    entity_count: int
    entities: list[WorldEntityState] = field(default_factory=list)
    source: str = 'gazebo'
    note: str | None = None

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> 'WorldStateSnapshot':
        """Create a world snapshot from a JSON mapping."""
        entities = [WorldEntityState.from_mapping(item) for item in data.get('entities', [])]
        return cls(
            available=bool(data.get('available', False)),
            world_name=str(data.get('world_name', '')),
            simulation_time_sec=_optional_float(data.get('simulation_time_sec')),
            entity_count=int(data.get('entity_count', len(entities))),
            entities=entities,
            source=str(data.get('source', 'gazebo')),
            note=data.get('note'),
        )


@dataclass
class RobotStateSnapshot:
    """Latest Dr.QP robot state snapshot."""

    timestamp: str
    available: bool
    simulation_running: bool
    lifecycle_state: str | None
    world_name: str | None
    simulation_time_sec: float | None
    robot_pose: Pose | None
    joint_states: dict[str, JointStateValue] = field(default_factory=dict)
    note: str | None = None

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> 'RobotStateSnapshot':
        """Create a robot state snapshot from a JSON mapping."""
        joint_states = {
            name: JointStateValue.from_mapping(value)
            for name, value in data.get('joint_states', {}).items()
        }
        return cls(
            timestamp=str(data.get('timestamp', '')),
            available=bool(data.get('available', False)),
            simulation_running=bool(data.get('simulation_running', False)),
            lifecycle_state=data.get('lifecycle_state'),
            world_name=data.get('world_name'),
            simulation_time_sec=_optional_float(data.get('simulation_time_sec')),
            robot_pose=Pose.from_mapping(data.get('robot_pose')),
            joint_states=joint_states,
            note=data.get('note'),
        )


@dataclass
class LifecycleActionResult:
    """Result of a robot lifecycle action."""

    action: str
    state_before: str | None
    state_after: str | None
    simulation_was_started: bool
    simulation_running: bool
    message: str
    log_path: str | None = None


@dataclass
class RecordingStatus:
    """Current robot state recording status."""

    active: bool
    sample_interval_sec: float | None
    sample_count: int
    started_at: str | None
    message: str


@dataclass
class RecordedRobotStates:
    """Recorded robot state samples."""

    started_at: str
    stopped_at: str
    sample_interval_sec: float
    sample_count: int
    samples: list[RobotStateSnapshot]


def _optional_float(value: Any) -> float | None:
    """Convert a value to float or None."""
    if value is None:
        return None
    return float(value)


def _optional_int(value: Any) -> int | None:
    """Convert a value to int or None."""
    if value is None:
        return None
    return int(value)
