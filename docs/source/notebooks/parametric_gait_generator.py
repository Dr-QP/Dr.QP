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

from enum import auto, Enum

from geometry import Point3D
from models import HexapodLeg
import numpy as np


class GaitType(Enum):
    ripple = auto()
    wave = auto()
    tripod = auto()


class ParametricGaitGenerator:
    """
    Parametric gait generator for hexapod robot.

    Generates leg tip offsets in right-hand coordinate system (x-forward, y-left, z-up).

    Parameters:
    -----------
        step_length: Length of each step in meters
        step_height: Height of leg lift in meters
    """

    class GaitPhaseParams:
        """
        Parameters for a specific gait.

        Parameters:
        -----------
            swing_phase_start_offsets: Dictionary of swing phase offsets per leg
            swing_duration: Duration of the swing phase
        """

        def __init__(
            self,
            swing_duration: float,
            swing_phase_start_offsets: dict[HexapodLeg, float],
        ):
            self.swing_phase_start_offsets = swing_phase_start_offsets
            self.swing_duration = swing_duration

    def __init__(self, step_length=1, step_height=1, gait=GaitType.wave):
        super().__init__()

        self.step_length = step_length
        self.step_height = step_height
        self.current_gait = gait

        # Gait params - START
        self.gaits = {
            GaitType.wave: self.GaitPhaseParams(
                swing_duration=1 / 6,
                swing_phase_start_offsets={
                    HexapodLeg.right_back: 0,
                    HexapodLeg.right_middle: 1 / 6,
                    HexapodLeg.right_front: 2 / 6,
                    HexapodLeg.left_back: 3 / 6,
                    HexapodLeg.left_middle: 4 / 6,
                    HexapodLeg.left_front: 5 / 6,
                },
            ),
            GaitType.ripple: self.GaitPhaseParams(
                swing_duration=1 / 3,
                swing_phase_start_offsets={
                    HexapodLeg.right_back: 0,
                    HexapodLeg.right_middle: 2 / 6,
                    HexapodLeg.right_front: 4 / 6,
                    HexapodLeg.left_back: 3 / 6,
                    HexapodLeg.left_middle: 5 / 6,
                    HexapodLeg.left_front: 1 / 6,
                },
            ),
            GaitType.tripod: self.GaitPhaseParams(
                swing_duration=1 / 2,
                swing_phase_start_offsets={
                    HexapodLeg.right_back: 0,
                    HexapodLeg.right_middle: 1 / 2,
                    HexapodLeg.right_front: 0,
                    HexapodLeg.left_back: 1 / 2,
                    HexapodLeg.left_middle: 0,
                    HexapodLeg.left_front: 1 / 2,
                },
            ),
        }
        # Gait params - END

    # Parametric function - START
    def get_offsets_at_phase_for_leg(self, leg, phase) -> Point3D:
        gait = self.gaits[self.current_gait]
        leg_phase = phase - gait.swing_phase_start_offsets[leg]
        leg_phase %= 1

        half_step = self.step_length / 2
        if leg_phase < gait.swing_duration:
            # Swing phase - leg in air moving forward
            t = np.interp(leg_phase, [0, gait.swing_duration], [0, 1])
            x_offset = np.interp(leg_phase, [0, gait.swing_duration], [-half_step, half_step])
            z_offset = np.sin(t * np.pi) * self.step_height
        else:
            # Stance phase - leg on ground moving backward
            x_offset = np.interp(leg_phase, [gait.swing_duration, 1], [half_step, -half_step])
            z_offset = 0  # On ground

        return Point3D(
            [
                x_offset,
                0,
                z_offset,
            ]
        )
        # Parametric function - END
