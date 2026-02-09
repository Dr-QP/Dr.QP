# Copyright (c) 2017-present Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from unittest.mock import Mock

from control_msgs.action import FollowJointTrajectory
from drqp_brain.joint_trajectory_builder import (
    JointTrajectoryBuilder,
    kFemurOffsetAngle,
    kTibiaOffsetAngle,
)
from drqp_brain.models import HexapodModel
import numpy as np
import pytest
import rclpy.time
import trajectory_msgs.msg


@pytest.fixture
def hexapod():
    """Create a real hexapod model for testing."""
    return HexapodModel()


@pytest.fixture
def mock_publisher():
    """Create a mock publisher."""
    return Mock()


@pytest.fixture
def mock_action_client():
    """Create a mock action client."""
    client = Mock()
    client.send_goal_async.return_value = Mock()
    return client


@pytest.fixture
def mock_node():
    """Create a mock ROS2 node."""
    node = Mock()
    logger = Mock()
    node.get_logger.return_value = logger
    return node


@pytest.fixture
def trajectory_builder(hexapod):
    """Create a JointTrajectoryBuilder instance."""
    return JointTrajectoryBuilder(hexapod)


class TestJointTrajectoryBuilder:
    """Test the JointTrajectoryBuilder class."""

    def test_initialization(self, trajectory_builder, hexapod):
        """Test that JointTrajectoryBuilder initializes correctly."""
        assert trajectory_builder.hexapod is hexapod
        assert trajectory_builder.points == []

    def test_add_point_from_hexapod(self, trajectory_builder):
        """Test adding a point from hexapod state."""
        trajectory_builder.add_point_from_hexapod(1.0, effort=0.8)

        assert len(trajectory_builder.points) == 1
        point = trajectory_builder.points[0]

        # Check that positions are in radians
        assert all(isinstance(pos, float) for pos in point.positions)
        # Check that we have 18 joints (6 legs * 3 joints)
        assert len(point.positions) == 18
        assert len(point.effort) == 18
        assert all(effort == pytest.approx(0.8) for effort in point.effort)

    def test_add_point_from_hexapod_with_joint_mask(self, trajectory_builder):
        """Test adding a point with joint mask."""
        joint_mask = ['coxa', 'femur']
        trajectory_builder.add_point_from_hexapod(1.0, effort=0.5, joint_mask=joint_mask)

        point = trajectory_builder.points[0]
        # Should have effort=0.5 for coxa and femur, 0.0 for tibia
        expected_efforts = [0.5, 0.5, 0.0] * 6  # 6 legs
        assert point.effort == pytest.approx(expected_efforts)

    def test_joint_names_generation(self, trajectory_builder):
        """Test that joint names are generated correctly."""
        trajectory_builder.add_point_from_hexapod(1.0)

        # Check that joint names follow the expected pattern
        leg_names = [leg.label.name for leg in trajectory_builder.hexapod.legs]

        expected_names = []
        for leg_name in leg_names:
            for joint_type in ['coxa', 'femur', 'tibia']:
                expected_names.append(f'dr_qp/{leg_name}_{joint_type}')

        assert trajectory_builder.joint_names == expected_names

    def test_angle_offsets_applied(self, trajectory_builder):
        """Test that femur and tibia offsets are applied correctly."""
        # Set known angles for testing
        trajectory_builder.hexapod.forward_kinematics(0, 10, 20)

        trajectory_builder.add_point_from_hexapod(1.0)
        point = trajectory_builder.points[0]

        # Check that offsets are applied (every 3rd position for each joint type)
        for i in range(6):  # 6 legs
            coxa_pos = point.positions[i * 3]
            femur_pos = point.positions[i * 3 + 1]
            tibia_pos = point.positions[i * 3 + 2]

            assert coxa_pos == pytest.approx(np.radians(0.0))
            assert femur_pos == pytest.approx(np.radians(10.0 + kFemurOffsetAngle))
            assert tibia_pos == pytest.approx(np.radians(20.0 + kTibiaOffsetAngle))

    def test_add_point_direct(self, trajectory_builder):
        """Test adding a point directly with positions and effort."""
        positions = [0.1, 0.2, 0.3]
        effort = [0.5, 0.6, 0.7]
        seconds = 2.5

        trajectory_builder.add_point(positions, effort, seconds)

        assert len(trajectory_builder.points) == 1
        point = trajectory_builder.points[0]
        assert point.positions.tolist() == pytest.approx(positions)
        assert point.effort.tolist() == pytest.approx(effort)
        assert point.time_from_start == rclpy.time.Duration(seconds=seconds).to_msg()

    def test_publish(self, trajectory_builder, mock_publisher):
        """Test publishing trajectory message."""
        trajectory_builder.add_point_from_hexapod(1.0)
        trajectory_builder.publish(mock_publisher)

        mock_publisher.publish.assert_called_once()
        published_msg = mock_publisher.publish.call_args[0][0]

        assert isinstance(published_msg, trajectory_msgs.msg.JointTrajectory)
        assert published_msg.joint_names == trajectory_builder.joint_names
        assert published_msg.points == trajectory_builder.points

    def test_publish_action(self, trajectory_builder, mock_action_client, mock_node):
        """Test publishing trajectory as action goal."""
        result_callback = Mock()
        trajectory_builder.add_point_from_hexapod(1.0)

        # Mock the goal handle future
        goal_handle = Mock()
        goal_handle.accepted = True
        result_future = Mock()
        goal_handle.get_result_async.return_value = result_future

        goal_handle_future = Mock()
        goal_handle_future.result.return_value = goal_handle
        mock_action_client.send_goal_async.return_value = goal_handle_future

        trajectory_builder.publish_action(mock_action_client, mock_node, result_callback)

        # Verify goal was sent
        mock_action_client.send_goal_async.assert_called_once()
        sent_goal = mock_action_client.send_goal_async.call_args[0][0]

        assert isinstance(sent_goal, FollowJointTrajectory.Goal)
        assert sent_goal.trajectory.joint_names == trajectory_builder.joint_names
        assert sent_goal.trajectory.points == trajectory_builder.points

    def test_publish_action_goal_rejected(self, trajectory_builder, mock_action_client, mock_node):
        """Test handling of rejected action goal."""
        result_callback = Mock()
        trajectory_builder.add_point_from_hexapod(1.0)

        # Mock rejected goal
        goal_handle = Mock()
        goal_handle.accepted = False

        goal_handle_future = Mock()
        goal_handle_future.result.return_value = goal_handle
        mock_action_client.send_goal_async.return_value = goal_handle_future

        trajectory_builder.publish_action(mock_action_client, mock_node, result_callback)

        # Simulate callback execution
        goal_response_callback = goal_handle_future.add_done_callback.call_args[0][0]
        goal_response_callback(goal_handle_future)

        # Verify error was logged
        mock_node.get_logger().error.assert_called_with('Goal rejected')

    def test_multiple_points(self, trajectory_builder):
        """Test adding multiple trajectory points."""
        trajectory_builder.add_point_from_hexapod(1.0, effort=0.5)
        trajectory_builder.add_point_from_hexapod(2.0, effort=0.8)
        trajectory_builder.add_point_from_hexapod(3.0, effort=1.0)

        assert len(trajectory_builder.points) == 3

        # Check timing
        assert (
            trajectory_builder.points[0].time_from_start
            == rclpy.time.Duration(seconds=1.0).to_msg()
        )
        assert (
            trajectory_builder.points[1].time_from_start
            == rclpy.time.Duration(seconds=2.0).to_msg()
        )
        assert (
            trajectory_builder.points[2].time_from_start
            == rclpy.time.Duration(seconds=3.0).to_msg()
        )

        # Check efforts
        assert all(e == 0.5 for e in trajectory_builder.points[0].effort)
        assert all(e == 0.8 for e in trajectory_builder.points[1].effort)
        assert all(e == 1.0 for e in trajectory_builder.points[2].effort)
