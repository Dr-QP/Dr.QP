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

from drqp_brain.robot_state.robot_state_machine import RobotStateMachine
import pytest


class TestRobotStateMachine:
    """Test the RobotStateMachine class."""

    @pytest.fixture
    def state_machine(self):
        return RobotStateMachine()

    def test_initial_state(self, state_machine):
        assert state_machine.current_state == state_machine.torque_off
        # img_path = './robot_state_machine.png'
        # state_machine._graph().write_png(img_path)

    def test_should_turn_on_from_initialization(self, state_machine):
        state_machine.send('initialize')
        assert state_machine.current_state == state_machine.initializing

        state_machine.send('turn_on', state_machine)
        assert state_machine.current_state == state_machine.torque_on

    def test_should_finalize_from_turned_on(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('turn_on')
        assert state_machine.current_state == state_machine.torque_on

        state_machine.send('finalize')
        assert state_machine.current_state == state_machine.finalizing

    def test_should_done_from_finalizing(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('turn_on')
        state_machine.send('finalize')
        assert state_machine.current_state == state_machine.finalizing

        state_machine.send('done')
        assert state_machine.current_state == state_machine.finalized
        assert state_machine.is_finished

    def test_should_turn_off_from_initialization(self, state_machine):
        state_machine.send('initialize')
        assert state_machine.current_state == state_machine.initializing

        state_machine.send('turn_off')
        assert state_machine.current_state == state_machine.torque_off

    def test_should_turn_off_from_turned_on(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('turn_on')
        assert state_machine.current_state == state_machine.torque_on

        state_machine.send('turn_off')
        assert state_machine.current_state == state_machine.torque_off

    def test_should_turn_off_from_finalizing(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('turn_on')
        state_machine.send('finalize')
        assert state_machine.current_state == state_machine.finalizing

        state_machine.send('turn_off')
        assert state_machine.current_state == state_machine.torque_off
