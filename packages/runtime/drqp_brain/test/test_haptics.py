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

from drqp_brain.haptics import (
    control_mode_feedback_pattern,
    gait_feedback_pattern,
    HapticFeedbackScheduler,
    LEFT_RUMBLE_CHANNEL_ID,
    RIGHT_RUMBLE_CHANNEL_ID,
)
import pytest


class _FakeControlMode:
    """Simple stand-in for control mode enums."""

    def __init__(self, name: str):
        self.name = name


def test_gait_feedback_pattern_uses_left_channel_pulse_counts():
    """Gait selections should map to left-channel pulse counts."""
    tripod = gait_feedback_pattern('tripod')
    ripple = gait_feedback_pattern('ripple')
    wave = gait_feedback_pattern('wave')

    assert tripod.channel_id == LEFT_RUMBLE_CHANNEL_ID
    assert ripple.channel_id == LEFT_RUMBLE_CHANNEL_ID
    assert wave.channel_id == LEFT_RUMBLE_CHANNEL_ID
    assert tripod.pulse_count == 1
    assert ripple.pulse_count == 2
    assert wave.pulse_count == 3


def test_control_mode_feedback_pattern_uses_working_rumble_channel():
    """Control modes should preserve pulse mapping and repeat groups."""
    walk = control_mode_feedback_pattern(_FakeControlMode('Walk'))
    body_position = control_mode_feedback_pattern(_FakeControlMode('BodyPosition'))
    body_rotation = control_mode_feedback_pattern(_FakeControlMode('BodyRotation'))

    assert walk.channel_id == RIGHT_RUMBLE_CHANNEL_ID
    assert body_position.channel_id == RIGHT_RUMBLE_CHANNEL_ID
    assert body_rotation.channel_id == RIGHT_RUMBLE_CHANNEL_ID
    assert walk.pulse_count == 1
    assert body_position.pulse_count == 2
    assert body_rotation.pulse_count == 3
    assert walk.repeat_count == 3
    assert body_position.repeat_count == 3
    assert body_rotation.repeat_count == 3
    assert walk.repeat_gap_duration > 0.04


def test_scheduler_repeats_control_mode_groups_with_longer_group_gap():
    """Control-mode groups should repeat three times with a longer inter-group gap."""
    current_time = [50.0]
    scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])

    commands = scheduler.schedule(
        control_mode_feedback_pattern(_FakeControlMode('BodyPosition'))
    )

    active_commands = [command for command in commands if command.intensity > 0.0]
    assert len(active_commands) == 6
    assert active_commands[0].due_at == pytest.approx(50.0, abs=1e-7)
    assert active_commands[1].due_at == pytest.approx(50.2, abs=1e-7)
    assert active_commands[2].due_at == pytest.approx(50.6, abs=1e-7)
    assert active_commands[3].due_at == pytest.approx(50.8, abs=1e-7)
    assert active_commands[4].due_at == pytest.approx(51.2, abs=1e-7)
    assert active_commands[5].due_at == pytest.approx(51.4, abs=1e-7)


def test_scheduler_returns_on_off_commands_for_each_pulse():
    """A pulse pattern should expand into alternating on/off commands."""
    current_time = [10.0]
    scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])

    commands = scheduler.schedule(gait_feedback_pattern('ripple'))

    assert len(commands) == 8
    assert [command.intensity for command in commands] == [
        0.8, 0.0, 0.8, 0.0, 0.8, 0.0, 0.8, 0.0,
    ]
    assert [command.channel_id for command in commands] == [LEFT_RUMBLE_CHANNEL_ID] * 8
    assert commands[0].due_at == pytest.approx(10.0, abs=1e-7)
    assert commands[1].due_at == pytest.approx(10.15, abs=1e-7)
    assert commands[2].due_at == pytest.approx(10.2, abs=1e-7)
    assert commands[3].due_at == pytest.approx(10.35, abs=1e-7)
    assert commands[4].due_at == pytest.approx(10.6, abs=1e-7)
    assert commands[5].due_at == pytest.approx(10.75, abs=1e-7)
    assert commands[6].due_at == pytest.approx(10.8, abs=1e-7)
    assert commands[7].due_at == pytest.approx(10.95, abs=1e-7)


def test_scheduler_is_idempotent_for_same_finalized_state():
    """Repeated selection of the active state should not schedule more feedback."""
    current_time = [20.0]
    scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])
    tripod_pattern = gait_feedback_pattern('tripod')

    assert len(scheduler.schedule(tripod_pattern)) == 4
    assert scheduler.schedule(tripod_pattern) == []


def test_scheduler_applies_channel_cooldown_before_next_pattern():
    """Rapid updates on one channel should be delayed by the cooldown window."""
    current_time = [30.0]
    scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])

    scheduler.schedule(gait_feedback_pattern('tripod'))
    current_time[0] = 30.02
    commands = scheduler.schedule(gait_feedback_pattern('ripple'))

    assert commands[0].due_at == pytest.approx(30.65, abs=1e-7)
    assert commands[1].due_at == pytest.approx(30.8, abs=1e-7)


def test_scheduler_reset_channel_allows_resending_same_state():
    """
    After reset_channel, re-selecting the same state should produce.

    Fresh
    feedback commands (models backend reconnect scenario).
    """
    current_time = [40.0]
    scheduler = HapticFeedbackScheduler(clock=lambda: current_time[0])
    tripod_pattern = gait_feedback_pattern('tripod')

    first = scheduler.schedule(tripod_pattern)
    assert len(first) == 4

    # Simulate discarding due to missing backend and resetting channel
    scheduler.reset_channel(tripod_pattern.channel_id)

    # Same state should now produce commands again
    second = scheduler.schedule(tripod_pattern)
    assert len(second) == 4
