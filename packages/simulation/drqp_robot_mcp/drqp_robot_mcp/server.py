"""FastMCP server exposing Dr.QP robot tools."""

from __future__ import annotations

import atexit

from mcp.server.fastmcp import FastMCP

from .controller import RobotMcpController

mcp = FastMCP(
    'Dr.QP Robot MCP',
    instructions=(
        'Use the simulation, robot, and system namespaces to inspect runtime '
        'state, control the Dr.QP robot, and subscribe to state streams from '
        'the current ROS 2 environment.'
    ),
)
controller = RobotMcpController()
atexit.register(controller.close)


@mcp.tool(name='simulation.start')
def simulation_start(timeout_sec: float = 120.0) -> dict[str, object]:
    """Start the simulation runtime when it is available."""
    return controller.start_simulation(timeout_sec=timeout_sec)


@mcp.tool(name='simulation.stop')
def simulation_stop(timeout_sec: float = 120.0) -> dict[str, object]:
    """Stop the simulation runtime when it is active."""
    return controller.stop_simulation(timeout_sec=timeout_sec)


@mcp.tool(name='simulation.state')
def simulation_state(timeout_sec: float = 10.0) -> dict[str, object]:
    """Return the latest simulation runtime snapshot."""
    return controller.get_simulation_state(timeout_sec=timeout_sec)


@mcp.tool(name='simulation.robot_state')
def simulation_robot_state(timeout_sec: float = 10.0) -> dict[str, object]:
    """Return the latest simulated robot state snapshot."""
    return controller.get_simulation_robot_state(timeout_sec=timeout_sec)


@mcp.tool(name='simulation.world_state')
def simulation_world_state(timeout_sec: float = 10.0) -> dict[str, object]:
    """Return the latest simulated world state snapshot."""
    return controller.get_simulation_world_state(timeout_sec=timeout_sec)


@mcp.tool(name='robot.boot')
def robot_boot(timeout_sec: float = 120.0) -> dict[str, object]:
    """Boot the robot to the torque-enabled lifecycle state."""
    return _robot_lifecycle_result(controller.boot_up(timeout_sec=timeout_sec))


@mcp.tool(name='robot.shutdown')
def robot_shutdown(timeout_sec: float = 120.0) -> dict[str, object]:
    """Shutdown the robot through its lifecycle state machine."""
    return _robot_lifecycle_result(controller.shut_down(timeout_sec=timeout_sec))


@mcp.tool(name='robot.state')
def robot_state(timeout_sec: float = 10.0) -> dict[str, object]:
    """Return the latest robot lifecycle and joint-state snapshot."""
    return controller.get_robot_namespace_state(timeout_sec=timeout_sec)


@mcp.tool(name='robot.move')
def robot_move(
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
) -> dict[str, object]:
    """Publish a normalized motion command for robot walking or posture."""
    return _motion_command_state(
        controller.send_motion_command(
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
    )


@mcp.tool(name='robot.stop')
def robot_stop() -> dict[str, object]:
    """Publish a zeroed motion command to stop robot motion."""
    return _motion_command_state(controller.stop_motion())


@mcp.tool(name='robot.walk_for_duration')
def robot_walk_for_duration(
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
) -> dict[str, object]:
    """Repeat a walking command for a bounded duration."""
    return _motion_sequence_result(
        controller.walk_for_duration(
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
    )


@mcp.tool(name='robot.recording.start')
def robot_recording_start(sample_interval_sec: float = 0.5) -> dict[str, object]:
    """Start recording robot state snapshots."""
    return _recording_status(controller.start_recording(sample_interval_sec))


@mcp.tool(name='robot.recording.stop')
def robot_recording_stop() -> dict[str, object]:
    """Stop recording and return the captured robot state snapshots."""
    recorded = controller.stop_recording()
    return {
        'started_at': recorded.started_at,
        'stopped_at': recorded.stopped_at,
        'sample_interval_sec': recorded.sample_interval_sec,
        'sample_count': recorded.sample_count,
        'samples': [_robot_state_payload(sample) for sample in recorded.samples],
    }


@mcp.tool(name='robot.recording.status')
def robot_recording_status() -> dict[str, object]:
    """Return whether robot-state recording is active."""
    return _recording_status(controller.get_recording_status())


@mcp.tool(name='system.state')
def system_state(timeout_sec: float = 10.0) -> dict[str, object]:
    """Return the latest system runtime and transport health snapshot."""
    return controller.get_system_state(timeout_sec=timeout_sec)


@mcp.resource('drqp://simulation.state.stream')
def simulation_state_stream() -> dict[str, object]:
    """Return the latest simulation stream event envelope."""
    return controller.stream_simulation_state()


@mcp.resource('drqp://simulation.robot_state.stream')
def simulation_robot_state_stream() -> dict[str, object]:
    """Return the latest simulated robot stream event envelope."""
    return controller.stream_simulation_robot_state()


@mcp.resource('drqp://simulation.world_state.stream')
def simulation_world_state_stream() -> dict[str, object]:
    """Return the latest world-state stream event envelope."""
    return controller.stream_simulation_world_state()


@mcp.resource('drqp://robot.state.stream')
def robot_state_stream() -> dict[str, object]:
    """Return the latest robot-state stream event envelope."""
    return controller.stream_robot_state()


@mcp.resource('drqp://system.state.stream')
def system_state_stream() -> dict[str, object]:
    """Return the latest system-state stream event envelope."""
    return controller.stream_system_state()


def _robot_lifecycle_result(result: object) -> dict[str, object]:
    """Adapt the controller lifecycle result to the spec payload shape."""
    return {
        'action': getattr(result, 'action'),
        'state_before': getattr(result, 'state_before'),
        'state_after': getattr(result, 'state_after'),
        'active': getattr(result, 'state_after') == 'torque_on',
        'message': getattr(result, 'message'),
        'note': None,
    }


def _motion_command_state(result: object) -> dict[str, object]:
    """Adapt the controller motion command result to the spec payload shape."""
    latest_motion_command = getattr(controller, '_latest_motion_command', None) or {}
    return {
        'stride_direction': {
            'x': result.stride_direction.x,
            'y': result.stride_direction.y,
            'z': result.stride_direction.z,
        },
        'rotation_speed': result.rotation_speed,
        'body_translation': {
            'x': result.body_translation.x,
            'y': result.body_translation.y,
            'z': result.body_translation.z,
        },
        'body_rotation': {
            'roll': result.body_rotation.x,
            'pitch': result.body_rotation.y,
            'yaw': result.body_rotation.z,
        },
        'gait_type': result.gait_type,
        'timestamp': latest_motion_command.get('timestamp'),
        'note': latest_motion_command.get('note', result.message),
    }


def _motion_sequence_result(result: object) -> dict[str, object]:
    """Adapt the controller motion sequence result to the spec payload shape."""
    return {
        'duration_sec': result.duration_sec,
        'publish_hz': result.publish_hz,
        'publish_count': result.publish_count,
        'stop_command_sent': result.stop_command_sent,
        'command': _motion_command_state(result.command),
        'message': result.message,
    }


def _recording_status(result: object) -> dict[str, object]:
    """Adapt the controller recording status to the spec payload shape."""
    return {
        'active': result.active,
        'sample_interval_sec': result.sample_interval_sec,
        'sample_count': result.sample_count,
        'started_at': result.started_at,
        'message': result.message,
    }


def _robot_state_payload(result: object) -> dict[str, object]:
    """Adapt an internal robot snapshot to the spec robot.state payload."""
    joint_states = {}
    for name, value in result.joint_states.items():
        joint_states[name] = {
            'position': value.position,
            'velocity': value.velocity,
            'effort': value.effort,
        }
    return {
        'timestamp': result.timestamp,
        'available': result.available,
        'lifecycle_state': result.lifecycle_state,
        'active': result.lifecycle_state == 'torque_on',
        'joint_states': joint_states,
        'latest_motion_command': getattr(controller, '_latest_motion_command', None),
        'note': result.note,
    }


def main() -> None:
    """Run the MCP server over stdio."""
    try:
        mcp.run()
    finally:
        controller.close()


if __name__ == '__main__':
    main()
