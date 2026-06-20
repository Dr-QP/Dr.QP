"""Integration tests for the Dr.QP robot MCP runtime session."""

from __future__ import annotations

import time
from typing import Any

from drqp_robot_mcp import runtime
from drqp_robot_mcp.controller import RobotMcpController
import pytest


def _read_odom_pose(timeout_sec: float) -> Any | None:
    from nav_msgs.msg import Odometry
    import rclpy

    did_init = False
    if not rclpy.ok():
        rclpy.init()
        did_init = True

    node = rclpy.create_node('drqp_robot_mcp_runtime_integration_pose_reader')
    latest_pose: dict[str, Any | None] = {'value': None}

    def callback(message: Odometry) -> None:
        latest_pose['value'] = message.pose.pose

    node.create_subscription(Odometry, '/odom', callback, 10)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline and latest_pose['value'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        return latest_pose['value']
    finally:
        node.destroy_node()
        if did_init and rclpy.ok():
            rclpy.shutdown()


def _assert_pose_matches_odometry(snapshot_pose: dict[str, Any], odom_pose: Any) -> None:
    # Runtime snapshots and odom callbacks are sampled asynchronously, so use
    # realistic bounds instead of exact float equality.
    position_tolerance_m = 0.05
    orientation_tolerance = 0.01

    assert snapshot_pose['position']['x'] == pytest.approx(
        odom_pose.position.x,
        abs=position_tolerance_m,
    )
    assert snapshot_pose['position']['y'] == pytest.approx(
        odom_pose.position.y,
        abs=position_tolerance_m,
    )
    assert snapshot_pose['position']['z'] == pytest.approx(
        odom_pose.position.z,
        abs=position_tolerance_m,
    )
    assert snapshot_pose['orientation']['x'] == pytest.approx(
        odom_pose.orientation.x,
        abs=orientation_tolerance,
    )
    assert snapshot_pose['orientation']['y'] == pytest.approx(
        odom_pose.orientation.y,
        abs=orientation_tolerance,
    )
    assert snapshot_pose['orientation']['z'] == pytest.approx(
        odom_pose.orientation.z,
        abs=orientation_tolerance,
    )
    assert snapshot_pose['orientation']['w'] == pytest.approx(
        odom_pose.orientation.w,
        abs=orientation_tolerance,
    )


@pytest.mark.slow
@pytest.mark.timeout(300)
def test_runtime_robot_pose_stays_available_from_live_odom_bridge() -> None:
    """The runtime robot snapshot keeps exposing pose from the live /odom bridge."""
    controller = RobotMcpController()

    try:
        controller.boot_up(timeout_sec=120.0)

        first_odom_pose = _read_odom_pose(timeout_sec=10.0)
        assert first_odom_pose is not None

        first_snapshot = runtime.get_robot_state(
            world_name=controller.world_name,
            robot_name=controller.robot_name,
            timeout_sec=10.0,
        )

        assert first_snapshot['available'] is True
        assert first_snapshot['simulation_running'] is True
        assert first_snapshot['robot_pose'] is not None
        assert first_snapshot['simulation_time_sec'] is not None
        _assert_pose_matches_odometry(first_snapshot['robot_pose'], first_odom_pose)

        deadline = time.monotonic() + 15.0
        latest_snapshot: dict[str, Any] | None = None
        latest_odom_pose: Any | None = None

        while time.monotonic() < deadline:
            time.sleep(0.5)
            candidate_odom_pose = _read_odom_pose(timeout_sec=2.0)
            if candidate_odom_pose is None:
                continue

            candidate_snapshot = runtime.get_robot_state(
                world_name=controller.world_name,
                robot_name=controller.robot_name,
                timeout_sec=2.0,
            )
            if candidate_snapshot['robot_pose'] is None:
                continue
            if candidate_snapshot['simulation_time_sec'] is None:
                continue
            if candidate_snapshot['simulation_time_sec'] <= first_snapshot['simulation_time_sec']:
                continue

            latest_snapshot = candidate_snapshot
            latest_odom_pose = candidate_odom_pose
            break

        assert latest_snapshot is not None
        assert latest_odom_pose is not None
        assert latest_snapshot['available'] is True
        assert latest_snapshot['simulation_running'] is True
        assert latest_snapshot['robot_pose'] is not None
        _assert_pose_matches_odometry(latest_snapshot['robot_pose'], latest_odom_pose)
    finally:
        controller.stop_motion()
        controller.shut_down(timeout_sec=60.0)
        controller.stop_simulation(timeout_sec=60.0)
        controller.close()
        runtime.shutdown_default_ros_runtime()
