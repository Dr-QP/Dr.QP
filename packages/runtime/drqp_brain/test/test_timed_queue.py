# Copyright (c) 2017-2025 Anton Matosov
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

from drqp_brain.timed_queue import TimedQueue
import pytest
import rclpy


class TestTimedQueue:
    """Test the TimedQueue class."""

    @pytest.fixture
    def timed_queue(self):
        rclpy.init()
        node = rclpy.create_node('test_timed_queue')
        yield TimedQueue(node)
        rclpy.shutdown()

    def test_added_action_is_executed_immediately(self, timed_queue):
        """Check whether actions are executed immediately."""
        executed = False

        def action():
            nonlocal executed
            executed = True

        timed_queue.add(0.1, action)
        rclpy.spin_once(timed_queue.node, timeout_sec=0.1)
        assert executed

    def test_next_added_action_is_executed_after_delay(self, timed_queue):
        """Check whether actions are executed after the delay."""
        executed1 = False
        executed2 = False

        def action1():
            nonlocal executed1
            executed1 = True

        def action2():
            nonlocal executed2
            executed2 = True

        timed_queue.add(0.2, action1)
        timed_queue.add(0.2, action2)
        rclpy.spin_once(timed_queue.node, timeout_sec=0.05)
        assert executed1
        assert not executed2

        while timed_queue.timer is not None:
            rclpy.spin_once(timed_queue.node, timeout_sec=0.05)
        assert executed2

    def test_clear(self, timed_queue):
        """Check whether the queue is cleared."""
        executed1 = False
        executed2 = False

        def action1():
            nonlocal executed1
            executed1 = True

        def action2():
            nonlocal executed2
            executed2 = True

        timed_queue.add(0.01, action1)
        timed_queue.add(0.01, action2)
        timed_queue.clear()
        rclpy.spin_once(timed_queue.node, timeout_sec=0.05)
        assert not executed2

        assert timed_queue.timer is None
        assert len(timed_queue.queue) == 0
