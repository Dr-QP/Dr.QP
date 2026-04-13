#!/usr/bin/env python3
#
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

from collections import defaultdict
from dataclasses import dataclass
import time
from typing import Callable


LEFT_RUMBLE_CHANNEL_ID = 0
RIGHT_RUMBLE_CHANNEL_ID = 1

GAIT_FEEDBACK_PULSES = {
    'tripod': 1,
    'ripple': 2,
    'wave': 3,
}

CONTROL_MODE_FEEDBACK_PULSES = {
    'Walk': 1,
    'BodyPosition': 2,
    'BodyRotation': 3,
}


@dataclass(frozen=True)
class HapticPulsePattern:
    """Pulse definition for a single rumble channel."""

    channel_id: int
    pulse_count: int
    state_key: str
    intensity: float = 0.8
    repeat_count: int = 1
    repeat_gap_duration: float = 0.0


@dataclass(frozen=True)
class ScheduledFeedbackCommand:
    """Single scheduled rumble command."""

    due_at: float
    channel_id: int
    intensity: float


class HapticFeedbackScheduler:
    """
    Schedule non-blocking haptic pulse sequences.

    The scheduler is stateful so it can suppress duplicate feedback and enforce
    a small cooldown window per rumble channel.
    """

    def __init__(
        self,
        clock: Callable[[], float] = time.monotonic,
        pulse_on_duration: float = 0.15,
        pulse_gap_duration: float = 0.05,
        cooldown_window: float = 0.1,
    ):
        self._clock = clock
        self._pulse_on_duration = pulse_on_duration
        self._pulse_gap_duration = pulse_gap_duration
        self._cooldown_window = cooldown_window
        self._last_state_by_channel: dict[int, str] = {}
        self._next_feedback_at = defaultdict(float)

    def now(self) -> float:
        """Return the scheduler clock time."""
        return self._clock()

    def schedule(
        self, pattern: HapticPulsePattern
    ) -> list[ScheduledFeedbackCommand]:
        """
        Create the scheduled rumble commands for a feedback pattern.

        Duplicate selections for the same finalized state are ignored. New
        patterns on the same channel are delayed by a short cooldown window.
        """
        if pattern.pulse_count <= 0:
            return []

        if pattern.repeat_count <= 0:
            return []

        if (
            self._last_state_by_channel.get(pattern.channel_id)
            == pattern.state_key
        ):
            return []

        start_at = max(self.now(), self._next_feedback_at[pattern.channel_id])
        commands = []
        pulse_span = self._pulse_on_duration + self._pulse_gap_duration
        group_gap = max(pattern.repeat_gap_duration, self._pulse_gap_duration)

        for repeat_index in range(pattern.repeat_count):
            group_start = start_at + repeat_index * (
                (pattern.pulse_count * pulse_span) - self._pulse_gap_duration
                + group_gap
            )

            for pulse_index in range(pattern.pulse_count):
                pulse_start = group_start + (pulse_index * pulse_span)
                commands.append(
                    ScheduledFeedbackCommand(
                        due_at=pulse_start,
                        channel_id=pattern.channel_id,
                        intensity=pattern.intensity,
                    )
                )
                commands.append(
                    ScheduledFeedbackCommand(
                        due_at=pulse_start + self._pulse_on_duration,
                        channel_id=pattern.channel_id,
                        intensity=0.0,
                    )
                )

        self._last_state_by_channel[pattern.channel_id] = pattern.state_key
        self._next_feedback_at[pattern.channel_id] = (
            commands[-1].due_at + self._cooldown_window
        )

        return commands

    def reset_channel(self, channel_id: int) -> None:
        """
        Clear the remembered state for a channel.

        Call this when pending commands for a channel are discarded (e.g. due
        to a missing or failed haptics backend) so that a later re-selection
        of the same state can still emit feedback once the backend is available
        again.
        """
        self._last_state_by_channel.pop(channel_id, None)
        self._next_feedback_at.pop(channel_id, None)


def gait_feedback_pattern(gait_name: str) -> HapticPulsePattern:
    """Return the haptic pulse pattern for a gait selection."""
    return HapticPulsePattern(
        channel_id=LEFT_RUMBLE_CHANNEL_ID,
        pulse_count=GAIT_FEEDBACK_PULSES[gait_name],
        state_key=f'gait:{gait_name}',
        repeat_count=2,
        repeat_gap_duration=0.25,
    )


def control_mode_feedback_pattern(control_mode: object) -> HapticPulsePattern:
    """Return the haptic pulse pattern for a control mode selection."""
    control_mode_name = getattr(control_mode, 'name', str(control_mode))

    return HapticPulsePattern(
        channel_id=LEFT_RUMBLE_CHANNEL_ID,
        pulse_count=CONTROL_MODE_FEEDBACK_PULSES[control_mode_name],
        state_key=f'control_mode:{control_mode_name}',
        repeat_count=3,
        repeat_gap_duration=0.25,
    )
