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

"""Unit tests for MoveIt smoke-test graph readiness."""

from unittest.mock import MagicMock

import moveit_launch_smoke_test_support as smoke_support
import pytest


class FakeMonotonicClock:
    """Controllable wall clock for deadline and polling assertions."""

    def __init__(self) -> None:
        self.now = 100.0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        """Return the current fake monotonic time."""
        return self.now

    def sleep(self, duration: float) -> None:
        """Advance fake time while recording the requested sleep."""
        self.sleeps.append(duration)
        self.now += duration


def test_stack_readiness_polls_for_brain_with_shared_deadline(monkeypatch) -> None:
    """Wait for drqp_brain after the MoveIt service becomes ready."""
    clock = FakeMonotonicClock()
    client = MagicMock()
    node = MagicMock()
    node.create_client.return_value = client
    node.get_node_names.side_effect = [[], ['drqp_brain']]

    def wait_for_service(*, timeout_sec: float) -> bool:
        """Consume part of the shared deadline before service readiness."""
        assert timeout_sec == pytest.approx(5.0)
        clock.now += 2.0
        return True

    client.wait_for_service.side_effect = wait_for_service
    monkeypatch.setattr(smoke_support.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(smoke_support.time, 'sleep', clock.sleep)
    monkeypatch.setattr(smoke_support.rclpy, 'create_node', lambda _name: node)

    smoke_support.assert_moveit_stack_ready(timeout=5.0)

    assert clock.sleeps == [smoke_support.NODE_POLL_INTERVAL]
    client.destroy.assert_called_once_with()
    node.destroy_node.assert_called_once_with()


def test_stack_readiness_bounds_node_polling_by_remaining_time(monkeypatch) -> None:
    """Fail on the wall deadline without oversleeping for graph discovery."""
    clock = FakeMonotonicClock()
    client = MagicMock()
    node = MagicMock()
    node.create_client.return_value = client
    node.get_node_names.return_value = []

    def wait_for_service(*, timeout_sec: float) -> bool:
        """Leave less than one poll interval in the shared budget."""
        assert timeout_sec == pytest.approx(1.0)
        clock.now += 0.95
        return True

    client.wait_for_service.side_effect = wait_for_service
    monkeypatch.setattr(smoke_support.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(smoke_support.time, 'sleep', clock.sleep)
    monkeypatch.setattr(smoke_support.rclpy, 'create_node', lambda _name: node)

    with pytest.raises(AssertionError, match='drqp_brain node is not available'):
        smoke_support.assert_moveit_stack_ready(timeout=1.0)

    assert clock.sleeps == [pytest.approx(0.05)]
    client.destroy.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
