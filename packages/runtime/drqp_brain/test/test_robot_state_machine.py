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
from statemachine.exceptions import TransitionNotAllowed


class TestRobotStateMachine:
    """Test the RobotStateMachine class."""

    @pytest.fixture
    def state_machine(self):
        return RobotStateMachine()

    def test_initial_state(self, state_machine):
        assert state_machine.torque_off in state_machine.configuration
        # img_path = './robot_state_machine.png'
        # state_machine._graph().write_png(img_path)

    def test_should_initializing_done_from_initialization(self, state_machine):
        state_machine.send('initialize')
        assert state_machine.initializing in state_machine.configuration

        state_machine.send('initializing_done', state_machine)
        assert state_machine.torque_on in state_machine.configuration

    def test_should_finalize_from_turned_on(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        assert state_machine.torque_on in state_machine.configuration

        state_machine.send('finalize')
        assert state_machine.finalizing in state_machine.configuration

    def test_should_done_from_finalizing(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('finalize')
        assert state_machine.finalizing in state_machine.configuration

        state_machine.send('finalizing_done')
        assert state_machine.finalized in state_machine.configuration

    def test_should_turn_off_from_initialization(self, state_machine):
        state_machine.send('initialize')
        assert state_machine.initializing in state_machine.configuration

        state_machine.send('turn_off')
        assert state_machine.torque_off in state_machine.configuration

    def test_should_turn_off_from_turned_on(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        assert state_machine.torque_on in state_machine.configuration

        state_machine.send('turn_off')
        assert state_machine.torque_off in state_machine.configuration

    def test_should_turn_off_from_finalizing(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('finalize')
        assert state_machine.finalizing in state_machine.configuration

        state_machine.send('turn_off')
        assert state_machine.torque_off in state_machine.configuration

    def test_should_turn_off_from_finalized(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('finalize')
        state_machine.send('finalizing_done')
        assert state_machine.finalized in state_machine.configuration

        state_machine.send('turn_off')
        assert state_machine.torque_off in state_machine.configuration

    def test_should_reboot_servos_from_torque_on(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        assert state_machine.torque_on in state_machine.configuration

        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration

    def test_should_reboot_servos_from_initializing(self, state_machine):
        state_machine.send('initialize')
        assert state_machine.initializing in state_machine.configuration

        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration

    def test_should_reboot_servos_from_finalizing(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('finalize')
        assert state_machine.finalizing in state_machine.configuration

        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration

    def test_should_transition_to_torque_off_after_reboot(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration

        state_machine.send('servos_rebooting_done')
        assert state_machine.torque_off in state_machine.configuration

    def test_should_reboot_servos_from_torque_off(self, state_machine):
        assert state_machine.torque_off in state_machine.configuration

        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration

    def test_should_reboot_servos_from_finalized(self, state_machine):
        state_machine.send('initialize')
        state_machine.send('initializing_done')
        state_machine.send('finalize')
        state_machine.send('finalizing_done')
        assert state_machine.finalized in state_machine.configuration

        state_machine.send('reboot_servos')
        assert state_machine.servos_rebooting in state_machine.configuration


class TestKillSwitch:
    """Pin kill_switch_pressed, which combines turn_off | initialize."""

    PATHS_TO_STATE = {
        'torque_off': [],
        'initializing': ['initialize'],
        'torque_on': ['initialize', 'initializing_done'],
        'finalizing': ['initialize', 'initializing_done', 'finalize'],
        'finalized': ['initialize', 'initializing_done', 'finalize', 'finalizing_done'],
        'servos_rebooting': ['reboot_servos'],
    }

    @pytest.mark.parametrize(
        ('start', 'expected'),
        [
            ('torque_off', 'initializing'),
            ('initializing', 'torque_off'),
            ('torque_on', 'torque_off'),
            ('finalizing', 'torque_off'),
            ('finalized', 'initializing'),
        ],
    )
    def test_kill_switch_transition(self, start, expected):
        state_machine = RobotStateMachine()
        for event in self.PATHS_TO_STATE[start]:
            state_machine.send(event)
        assert getattr(state_machine, start) in state_machine.configuration

        state_machine.send('kill_switch_pressed')
        assert getattr(state_machine, expected) in state_machine.configuration

    def test_kill_switch_refused_while_servos_rebooting(self):
        state_machine = RobotStateMachine()
        for event in self.PATHS_TO_STATE['servos_rebooting']:
            state_machine.send(event)

        with pytest.raises(TransitionNotAllowed):
            state_machine.send('kill_switch_pressed')
        assert state_machine.servos_rebooting in state_machine.configuration
