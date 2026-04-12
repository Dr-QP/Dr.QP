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

import unittest

from drqp_brain.haptics import (
    LEFT_RUMBLE_CHANNEL_ID,
    RIGHT_RUMBLE_CHANNEL_ID,
    HapticFeedbackScheduler,
    control_mode_feedback_pattern,
    gait_feedback_pattern,
)


class _FakeControlMode:
    """Simple stand-in for control mode enums."""

    def __init__(self, name: str):
        self.name = name


class TestHaptics(unittest.TestCase):
    """Test haptic feedback mapping and scheduling."""

    def test_gait_feedback_pattern_uses_left_channel_pulse_counts(self):
        """Gait selections should map to left-channel pulse counts."""
        tripod = gait_feedback_pattern('tripod')
        ripple = gait_feedback_pattern('ripple')
        wave = gait_feedback_pattern('wave')

        self.assertEqual(tripod.channel_id, LEFT_RUMBLE_CHANNEL_ID)
        self.assertEqual(ripple.channel_id, LEFT_RUMBLE_CHANNEL_ID)
        self.assertEqual(wave.channel_id, LEFT_RUMBLE_CHANNEL_ID)
        self.assertEqual(tripod.pulse_count, 1)
        self.assertEqual(ripple.pulse_count, 2)
        self.assertEqual(wave.pulse_count, 3)

    def test_control_mode_feedback_pattern_uses_right_channel_pulse_counts(
        self,
    ):
        """Control mode selections should map to right-channel pulse counts."""
        walk = control_mode_feedback_pattern(_FakeControlMode('Walk'))
        body_position = control_mode_feedback_pattern(
            _FakeControlMode('BodyPosition')
        )
        body_rotation = control_mode_feedback_pattern(
            _FakeControlMode('BodyRotation')
        )

        self.assertEqual(walk.channel_id, RIGHT_RUMBLE_CHANNEL_ID)
        self.assertEqual(body_position.channel_id, RIGHT_RUMBLE_CHANNEL_ID)
        self.assertEqual(body_rotation.channel_id, RIGHT_RUMBLE_CHANNEL_ID)
        self.assertEqual(walk.pulse_count, 1)
        self.assertEqual(body_position.pulse_count, 2)
        self.assertEqual(body_rotation.pulse_count, 3)

    def test_scheduler_returns_on_off_commands_for_each_pulse(self):
        """A pulse pattern should expand into alternating on/off commands."""
        current_time = [10.0]
        scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])

        commands = scheduler.schedule(gait_feedback_pattern('ripple'))

        self.assertEqual(len(commands), 4)
        self.assertEqual(
            [command.intensity for command in commands],
            [0.8, 0.0, 0.8, 0.0],
        )
        self.assertEqual(
            [command.channel_id for command in commands],
            [0, 0, 0, 0],
        )
        self.assertAlmostEqual(commands[0].due_at, 10.0, places=7)
        self.assertAlmostEqual(commands[1].due_at, 10.06, places=7)
        self.assertAlmostEqual(commands[2].due_at, 10.1, places=7)
        self.assertAlmostEqual(commands[3].due_at, 10.16, places=7)

    def test_scheduler_is_idempotent_for_same_finalized_state(self):
        """
        Repeated selection of the active state should not schedule
        more feedback.
        """
        current_time = [20.0]
        scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])
        tripod_pattern = gait_feedback_pattern('tripod')

        self.assertEqual(len(scheduler.schedule(tripod_pattern)), 2)
        self.assertEqual(scheduler.schedule(tripod_pattern), [])

    def test_scheduler_applies_channel_cooldown_before_next_pattern(self):
        """
        Rapid updates on one channel should be delayed by the cooldown window.
        """
        current_time = [30.0]
        scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])

        scheduler.schedule(gait_feedback_pattern('tripod'))
        current_time[0] = 30.02
        commands = scheduler.schedule(gait_feedback_pattern('ripple'))

        self.assertAlmostEqual(commands[0].due_at, 30.16, places=7)
        self.assertAlmostEqual(commands[1].due_at, 30.22, places=7)


    def test_scheduler_reset_channel_allows_resending_same_state(self):
        """
        After reset_channel, re-selecting the same state should produce
        fresh feedback commands (models backend reconnect scenario).
        """
        current_time = [40.0]
        scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])
        tripod_pattern = gait_feedback_pattern('tripod')

        first = scheduler.schedule(tripod_pattern)
        self.assertEqual(len(first), 2)

        # Simulate discarding due to missing backend and resetting channel
        scheduler.reset_channel(tripod_pattern.channel_id)

        # Same state should now produce commands again
        second = scheduler.schedule(tripod_pattern)
        self.assertEqual(len(second), 2)


if __name__ == '__main__':
    unittest.main()
