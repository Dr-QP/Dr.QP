#!/usr/bin/env python3

from gait_generators import GaitGenerator
from models import HexapodLeg
import numpy as np
from point import Point3D

# Create a simple test gait generator
class TestGaitGenerator(GaitGenerator):
    def __init__(self, step_length=100, step_height=30):
        super().__init__()
        self.step_length = step_length
        self.step_height = step_height
        
        # Define phase offsets for each leg to create a tripod gait
        self.leg_phase_offsets = {
            HexapodLeg.left_front: 0.0,
            HexapodLeg.right_middle: 0.0,
            HexapodLeg.left_back: 0.0,
            HexapodLeg.right_front: 0.5,
            HexapodLeg.left_middle: 0.5,
            HexapodLeg.right_back: 0.5,
        }
        
    def get_offsets_at_phase_for_leg(self, leg, phase) -> Point3D:
        # Apply the leg's phase offset
        leg_phase = (phase + self.leg_phase_offsets[leg]) % 1.0
        
        half_step = self.step_length / 2
        
        # Swing phase is 40% of the cycle, stance phase is 60%
        swing_duration = 0.4
        stance_duration = 1.0 - swing_duration
        
        if leg_phase < swing_duration:
            # Swing phase - leg in air moving forward
            # Normalize phase to 0-1 range for swing
            swing_phase = leg_phase / swing_duration
            
            # Move from back to front
            x_offset = self.step_length * (-0.5 + swing_phase)
            
            # Parabolic trajectory for lifting leg
            z_offset = self.step_height * 4 * swing_phase * (1 - swing_phase)
        else:
            # Stance phase - leg on ground moving backward
            # Normalize phase to 0-1 range for stance
            stance_phase = (leg_phase - swing_duration) / stance_duration
            
            # Move from front to back
            x_offset = self.step_length * (0.5 - stance_phase)
            
            # On ground
            z_offset = 0
            
        return Point3D([x_offset, 0, z_offset])
    
    def get_offsets_at_phase(self, phase, **kwargs) -> dict:
        return {leg: self.get_offsets_at_phase_for_leg(leg, phase) for leg in self.all_legs}

if __name__ == "__main__":
    # Create and visualize the test gait
    gait_gen = TestGaitGenerator(step_length=100, step_height=30)
    gait_gen.visualize_continuous(steps=100, _subtitle='Test Gait with Black Swing Lines')
