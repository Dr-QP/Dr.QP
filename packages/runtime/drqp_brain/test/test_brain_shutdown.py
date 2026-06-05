# Copyright (c) 2026 Anton Matosov
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

from concurrent.futures import CancelledError
from unittest import mock

from drqp_brain.brain_node import HexapodBrain
import pytest
import rclpy


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.try_shutdown()


class PendingFuture:
    """A mock future that simulates a pending state and allows cancellation."""

    def __init__(self):
        self.cancel = mock.Mock(return_value=True)

    def done(self):
        return False


class AlwaysPendingFuture:
    """A mock future that never completes and tracks done() calls."""

    def __init__(self):
        self.done_calls = 0
        self.cancel = mock.Mock(return_value=True)
        self.result = mock.Mock()

    def done(self):
        self.done_calls += 1
        return False


class DoneFutureWithException:
    """A mock future that is already done and raises on result()."""

    def __init__(self, exception: Exception):
        self.cancel = mock.Mock(return_value=False)
        self.result = mock.Mock(side_effect=exception)

    def done(self):
        return True


def test_destroy_node_stops_walk_controller_before_destroying_ros_entities():
    brain = HexapodBrain()
    try:
        brain.loop_timer.cancel = mock.Mock()
        brain.walker.reset = mock.Mock()
        brain._last_published_foot_targets = ('left_front_leg', 0.0, 0.0, 0.0)

        with mock.patch('rclpy.node.Node.destroy_node', return_value=True) as destroy_super:
            brain.destroy_node()

        brain.loop_timer.cancel.assert_called_once_with()
        brain.walker.reset.assert_called_once_with()
        assert brain._last_published_foot_targets is None
        destroy_super.assert_called_once_with()
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


def test_destroy_node_cancels_pending_futures_before_destroying_ik_client():
    brain = HexapodBrain()
    try:
        future = PendingFuture()
        ik_client = mock.Mock()
        brain._HexapodBrain__ik_client = ik_client
        brain._track_future(future)

        with mock.patch('rclpy.node.Node.destroy_node', return_value=True):
            brain.destroy_node()

        future.cancel.assert_called_once_with()
        ik_client.destroy.assert_called_once_with()
        assert not brain._pending_futures
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


def test_destroy_node_continues_cleanup_when_client_destroy_raises():
    brain = HexapodBrain()
    try:
        trajectory_client = mock.Mock()
        ik_client = mock.Mock()
        trajectory_client.destroy.side_effect = RuntimeError('trajectory destroy failed')
        ik_client.destroy.side_effect = RuntimeError('ik destroy failed')
        brain._HexapodBrain__trajectory_client = trajectory_client
        brain._HexapodBrain__ik_client = ik_client

        with (
            mock.patch.object(brain, 'get_logger') as get_logger,
            mock.patch('rclpy.node.Node.destroy_node', return_value=True) as destroy_super,
        ):
            brain.destroy_node()

        assert get_logger.return_value.warning.call_count >= 2
        destroy_super.assert_called_once_with()
    finally:
        if rclpy.ok():
            rclpy.try_shutdown()


def test_discard_future_ignores_cancelled_future_exception_issue358():
    brain = HexapodBrain()
    try:
        future = mock.Mock()
        future.exception.side_effect = CancelledError()
        brain._pending_futures.add(future)

        brain._discard_future(future)

        assert future not in brain._pending_futures
    finally:
        brain.destroy_node()


def test_cancel_pending_futures_does_not_poll_pending_future_completion():
    brain = HexapodBrain()
    try:
        future = AlwaysPendingFuture()
        brain._pending_futures.add(future)

        brain._cancel_pending_futures()

        future.cancel.assert_called_once_with()
        future.result.assert_not_called()
        assert future.done_calls <= 2
        assert not brain._pending_futures
    finally:
        brain.destroy_node()


def test_cancel_pending_futures_logs_when_done_future_result_raises():
    brain = HexapodBrain()
    try:
        future = DoneFutureWithException(RuntimeError('boom'))
        brain._pending_futures.add(future)

        with mock.patch.object(brain, '_log_shutdown_warning') as shutdown_warning:
            brain._cancel_pending_futures()

        future.cancel.assert_not_called()
        future.result.assert_called_once_with()
        shutdown_warning.assert_called_once()
        assert 'Pending future finished with exception during cancel' in (
            shutdown_warning.call_args.args[0]
        )
        assert not brain._pending_futures
    finally:
        brain.destroy_node()
