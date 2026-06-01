"""FastMCP server exposing Dr.QP robot tools."""

from __future__ import annotations

import atexit

from mcp.server.fastmcp import FastMCP

from .controller import RobotMcpController
from .models import (
    LifecycleActionResult,
    MotionCommandResult,
    MotionSequenceResult,
    RecordedRobotStates,
    RecordingStatus,
    RobotStateSnapshot,
    WorldStateSnapshot,
)


mcp = FastMCP(
    'Dr.QP Robot MCP',
    instructions=(
        'Use these tools to manage the Dr.QP robot from the current ROS 2 '
        'environment, record robot state snapshots, and inspect Gazebo world '
        'poses when Gazebo is available.'
    ),
)
controller = RobotMcpController()
atexit.register(controller.close)


@mcp.tool()
def drqp_robot_boot_up(timeout_sec: float = 120.0) -> LifecycleActionResult:
    """Boot the Dr.QP robot to the `torque_on` lifecycle state."""
    return controller.boot_up(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_shut_down(timeout_sec: float = 120.0) -> LifecycleActionResult:
    """Shut the Dr.QP robot down through its lifecycle state machine."""
    return controller.shut_down(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_get_state(timeout_sec: float = 10.0) -> RobotStateSnapshot:
    """Return the latest robot lifecycle state, joint states, and robot pose."""
    return controller.get_robot_state(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_start_state_recording(
    sample_interval_sec: float = 0.5,
) -> RecordingStatus:
    """Start recording robot state snapshots at a fixed interval."""
    return controller.start_recording(sample_interval_sec=sample_interval_sec)


@mcp.tool()
def drqp_robot_stop_state_recording() -> RecordedRobotStates:
    """Stop robot state recording and return the recorded snapshots."""
    return controller.stop_recording()


@mcp.tool()
def drqp_robot_get_recording_status() -> RecordingStatus:
    """Return whether robot state recording is currently active."""
    return controller.get_recording_status()


@mcp.tool()
def drqp_robot_get_world_state(timeout_sec: float = 10.0) -> WorldStateSnapshot:
    """Return Gazebo world poses and simulation time when Gazebo is available."""
    return controller.get_world_state(timeout_sec=timeout_sec)


@mcp.tool()
def drqp_robot_send_motion_command(
    stride_x: float = 0.0,
    stride_y: float = 0.0,
    stride_z: float = 0.0,
    rotation_speed: float = 0.0,
    body_x: float = 0.0,
    body_y: float = 0.0,
    body_z: float = 0.0,
    body_roll: float = 0.0,
    body_pitch: float = 0.0,
    body_yaw: float = 0.0,
    gait_type: str = 'tripod',
) -> MotionCommandResult:
    """Publish a normalized joystick-like movement command for the robot."""
    return controller.send_motion_command(
        stride_x=stride_x,
        stride_y=stride_y,
        stride_z=stride_z,
        rotation_speed=rotation_speed,
        body_x=body_x,
        body_y=body_y,
        body_z=body_z,
        body_roll=body_roll,
        body_pitch=body_pitch,
        body_yaw=body_yaw,
        gait_type=gait_type,
    )


@mcp.tool()
def drqp_robot_stop_motion() -> MotionCommandResult:
    """Publish a zeroed motion command to stop walking or body motion."""
    return controller.stop_motion()


@mcp.tool()
def drqp_robot_walk_for_duration(
    duration_sec: float,
    publish_hz: float = 5.0,
    stop_after: bool = True,
    stride_x: float = 0.0,
    stride_y: float = 0.0,
    stride_z: float = 0.0,
    rotation_speed: float = 0.0,
    body_x: float = 0.0,
    body_y: float = 0.0,
    body_z: float = 0.0,
    body_roll: float = 0.0,
    body_pitch: float = 0.0,
    body_yaw: float = 0.0,
    gait_type: str = 'tripod',
) -> MotionSequenceResult:
    """Repeat a walking command for a fixed duration and optionally stop after."""
    return controller.walk_for_duration(
        duration_sec=duration_sec,
        publish_hz=publish_hz,
        stop_after=stop_after,
        stride_x=stride_x,
        stride_y=stride_y,
        stride_z=stride_z,
        rotation_speed=rotation_speed,
        body_x=body_x,
        body_y=body_y,
        body_z=body_z,
        body_roll=body_roll,
        body_pitch=body_pitch,
        body_yaw=body_yaw,
        gait_type=gait_type,
    )


def main() -> None:
    """Run the MCP server over stdio."""
    try:
        mcp.run()
    finally:
        controller.close()


if __name__ == '__main__':
    main()
