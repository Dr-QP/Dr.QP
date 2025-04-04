from enum import auto, Enum

from gait_generators import GaitGenerator
from models import HexapodLeg
import numpy as np
from point import Point3D


class GaitType(Enum):
    ripple = auto()
    wave = auto()
    tripod = auto()


class ParametricGaitGenerator(GaitGenerator):
    class GaitPhaseParams:
        def __init__(
            self,
            gait_type: GaitType,
            leg_phase_offsets: dict[HexapodLeg, float],
            swing_duration: float = None,
        ):
            self.gait_type = gait_type
            self.leg_phase_offsets = leg_phase_offsets
            self.swing_duration = swing_duration

    def __init__(self, step_length=1, step_height=1, gait=GaitType.wave):
        super().__init__()

        self.step_length = step_length
        self.step_height = step_height
        self.current_gait = gait

        self.gaits = {
            GaitType.wave: self.GaitPhaseParams(
                GaitType.wave,
                leg_phase_offsets={
                    HexapodLeg.right_back: 0,
                    HexapodLeg.right_middle: 1 / 6,
                    HexapodLeg.right_front: 2 / 6,
                    HexapodLeg.left_front: 3 / 6,
                    HexapodLeg.left_middle: 4 / 6,
                    HexapodLeg.left_back: 5 / 6,
                },
                swing_duration=1 / 6,
            ),
            GaitType.ripple: self.GaitPhaseParams(
                GaitType.ripple,
                leg_phase_offsets={
                    HexapodLeg.right_middle: 0,
                    HexapodLeg.left_front: 1 / 6,
                    HexapodLeg.right_back: 2 / 6,
                    HexapodLeg.left_middle: 3 / 6,
                    HexapodLeg.right_front: 4 / 6,
                    HexapodLeg.left_back: 5 / 6,
                },
                swing_duration=1 / 3,
            ),
            GaitType.tripod: self.GaitPhaseParams(
                GaitType.tripod,
                leg_phase_offsets={
                    HexapodLeg.left_front: 0,
                    HexapodLeg.right_middle: 0,
                    HexapodLeg.left_back: 0,
                    HexapodLeg.right_front: 1 / 2,
                    HexapodLeg.left_middle: 1 / 2,
                    HexapodLeg.right_back: 1 / 2,
                },
                swing_duration=1 / 2,
            ),
        }

    def get_offsets_at_phase_for_leg(self, leg, phase) -> Point3D:
        gait = self.gaits[self.current_gait]
        leg_phase = gait.leg_phase_offsets[leg] + phase
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
