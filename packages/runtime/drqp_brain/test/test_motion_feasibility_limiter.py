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

from drqp_brain.motion_feasibility_limiter import MotionFeasibilityLimiter
from drqp_kinematics.geometry import Point3D
import pytest


class FakeMotionHarness:

    def __init__(self, safe_scale_limit: float, search_iterations: int = 8):
        self.safe_scale_limit = safe_scale_limit
        self.state = {
            'phase': 0,
            'stride': Point3D([0, 0, 0]),
            'rotation': 0.0,
            'body_translation': Point3D([0, 0, 0]),
            'body_rotation': Point3D([0, 0, 0]),
        }
        self.solved_scales = []
        self.limiter = MotionFeasibilityLimiter(
            snapshot_motion_state=self.snapshot,
            restore_motion_state=self.restore,
            build_feet_target_window=self.build_window,
            solve_trajectory_targets=self.solve,
            motion_window_key=self.motion_key,
            search_iterations=search_iterations,
        )

    def snapshot(self):
        return {
            'phase': self.state['phase'],
            'stride': self.state['stride'].copy(),
            'rotation': self.state['rotation'],
            'body_translation': self.state['body_translation'].copy(),
            'body_rotation': self.state['body_rotation'].copy(),
        }

    def restore(self, state):
        self.state = {
            'phase': state['phase'],
            'stride': state['stride'].copy(),
            'rotation': state['rotation'],
            'body_translation': state['body_translation'].copy(),
            'body_rotation': state['body_rotation'].copy(),
        }

    def build_window(self, stride_direction, rotation_direction, body_translation, body_rotation):
        self.state['phase'] += 1
        self.state['stride'] = stride_direction.copy()
        self.state['rotation'] = rotation_direction
        self.state['body_translation'] = body_translation.copy()
        self.state['body_rotation'] = body_rotation.copy()
        return [
            [
                (
                    'fake_leg',
                    Point3D([stride_direction.x, stride_direction.y, rotation_direction]),
                )
            ]
        ]

    def solve(self, foot_target_window):
        scale = float(foot_target_window[0][0][1].x)
        self.solved_scales.append(scale)
        if scale <= self.safe_scale_limit:
            return [(foot_target_window[0], {'fake_joint': scale})]
        return None

    def motion_key(self, foot_target_window):
        target = foot_target_window[0][0][1]
        return (
            round(float(target.x), 6),
            round(float(target.y), 6),
            round(float(target.z), 6),
            round(float(self.state['body_translation'].z), 6),
        )


def test_accepts_full_motion_when_ik_allows_it():
    harness = FakeMotionHarness(safe_scale_limit=1.0)
    previous_state = harness.snapshot()

    result = harness.limiter.find_feasible_motion(
        previous_state=previous_state,
        stride_direction=Point3D([1, 0.4, 0.7]),
        rotation_direction=0.5,
        body_translation=Point3D([0.1, 0.2, 0.3]),
        body_rotation=Point3D([0.4, 0.5, 0.6]),
    )

    assert result.feasible
    assert result.candidate.scale == pytest.approx(1.0)
    assert harness.solved_scales == [1.0]
    assert harness.state['stride'] == Point3D([1.0, 0.4, 0.7])
    assert harness.state['rotation'] == pytest.approx(0.5)
    assert harness.state['body_translation'] == Point3D([0.1, 0.2, 0.3])
    assert harness.state['body_rotation'] == Point3D([0.4, 0.5, 0.6])


def test_binary_searches_largest_feasible_walking_scale():
    harness = FakeMotionHarness(safe_scale_limit=0.625, search_iterations=8)
    previous_state = harness.snapshot()

    result = harness.limiter.find_feasible_motion(
        previous_state=previous_state,
        stride_direction=Point3D([1, 0.5, 0.25]),
        rotation_direction=0.8,
        body_translation=Point3D([0.1, 0.2, 0.3]),
        body_rotation=Point3D([0.4, 0.5, 0.6]),
    )

    assert result.feasible
    assert result.candidate.scale <= 0.625
    assert result.candidate.scale == pytest.approx(0.625, abs=1 / 256)
    assert harness.state['stride'].x == pytest.approx(result.candidate.scale)
    assert harness.state['stride'].y == pytest.approx(0.5 * result.candidate.scale)
    assert harness.state['stride'].z == pytest.approx(0.25)
    assert harness.state['rotation'] == pytest.approx(0.8 * result.candidate.scale)
    assert harness.state['body_translation'] == Point3D([0.1, 0.2, 0.3])
    assert harness.state['body_rotation'] == Point3D([0.4, 0.5, 0.6])


def test_reports_body_pose_infeasible_when_zero_walking_fails():
    harness = FakeMotionHarness(safe_scale_limit=-0.1)
    previous_state = harness.snapshot()

    result = harness.limiter.find_feasible_motion(
        previous_state=previous_state,
        stride_direction=Point3D([1, 0, 0]),
        rotation_direction=1.0,
        body_translation=Point3D([0, 0, 0.3]),
        body_rotation=Point3D([0, 0, 0]),
    )

    assert not result.feasible
    assert result.body_pose_infeasible
    assert harness.state == previous_state
    assert harness.solved_scales == [1.0, 0.0]


def test_accepts_zero_walking_when_body_pose_is_feasible():
    harness = FakeMotionHarness(safe_scale_limit=0.0)
    previous_state = harness.snapshot()

    result = harness.limiter.find_feasible_motion(
        previous_state=previous_state,
        stride_direction=Point3D([1, 0.5, 0]),
        rotation_direction=0.8,
        body_translation=Point3D([0, 0, 0.3]),
        body_rotation=Point3D([0, 0.2, 0]),
    )

    assert result.feasible
    assert not result.body_pose_infeasible
    assert result.candidate.scale == pytest.approx(0.0)
    assert harness.state['stride'] == Point3D([0, 0, 0])
    assert harness.state['rotation'] == pytest.approx(0.0)
    assert harness.state['body_translation'] == Point3D([0, 0, 0.3])
    assert harness.state['body_rotation'] == Point3D([0, 0.2, 0])


def test_redundant_motion_is_detected_without_ik_solve():
    harness = FakeMotionHarness(safe_scale_limit=1.0)
    previous_state = harness.snapshot()
    last_published_key = (1.0, 0.0, 0.2, 0.3)

    result = harness.limiter.find_feasible_motion(
        previous_state=previous_state,
        stride_direction=Point3D([1, 0, 0]),
        rotation_direction=0.2,
        body_translation=Point3D([0, 0, 0.3]),
        body_rotation=Point3D([0, 0, 0]),
        last_published_motion_key=last_published_key,
    )

    assert result.redundant
    assert harness.solved_scales == []
    assert harness.state == previous_state
