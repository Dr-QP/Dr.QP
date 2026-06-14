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

from collections.abc import Callable
from dataclasses import dataclass

from drqp_kinematics.geometry import Point3D


@dataclass(frozen=True)
class FeasibleMotionCandidate:
    scale: float
    body_scale: float
    stride_direction: Point3D
    rotation_direction: float
    body_translation: Point3D
    body_rotation: Point3D
    feet_target_window: list
    trajectory_targets: list
    motion_key: tuple
    motion_state: dict


@dataclass(frozen=True)
class MotionFeasibilityResult:
    candidate: FeasibleMotionCandidate | None
    body_pose_infeasible: bool = False
    redundant: bool = False
    failed_scale: float | None = None

    @property
    def feasible(self) -> bool:
        return self.candidate is not None or self.redundant


class MotionFeasibilityLimiter:
    """Scale walking motion until the exact generated gait window is IK-feasible."""

    def __init__(
        self,
        snapshot_motion_state: Callable[[], dict],
        restore_motion_state: Callable[[dict], None],
        build_feet_target_window: Callable[[Point3D, float, Point3D, Point3D], list],
        solve_trajectory_targets: Callable[[list], list | None],
        motion_window_key: Callable[[list], tuple],
        search_iterations: int = 8,
    ):
        self._snapshot_motion_state = snapshot_motion_state
        self._restore_motion_state = restore_motion_state
        self._build_feet_target_window = build_feet_target_window
        self._solve_trajectory_targets = solve_trajectory_targets
        self._motion_window_key = motion_window_key
        self._search_iterations = search_iterations

    def find_feasible_motion(
        self,
        previous_state: dict,
        stride_direction: Point3D,
        rotation_direction: float,
        body_translation: Point3D,
        body_rotation: Point3D,
        last_published_motion_key: tuple | None = None,
    ) -> MotionFeasibilityResult:
        full_motion = self._try_candidate(
            previous_state,
            stride_direction,
            rotation_direction,
            body_translation,
            body_rotation,
            walking_scale=1.0,
            body_scale=1.0,
            last_published_motion_key=last_published_motion_key,
        )
        if full_motion.redundant or full_motion.candidate is not None:
            self._restore_accepted_state(full_motion)
            return full_motion

        zero_motion = self._try_candidate(
            previous_state,
            stride_direction,
            rotation_direction,
            body_translation,
            body_rotation,
            walking_scale=0.0,
            body_scale=1.0,
            last_published_motion_key=last_published_motion_key,
        )
        if zero_motion.redundant:
            return zero_motion
        if zero_motion.candidate is not None:
            low = 0.0
            high = 1.0
            best = zero_motion
            failed_scale = full_motion.failed_scale
            for _ in range(self._search_iterations):
                scale = (low + high) / 2.0
                candidate = self._try_candidate(
                    previous_state,
                    stride_direction,
                    rotation_direction,
                    body_translation,
                    body_rotation,
                    walking_scale=scale,
                    body_scale=1.0,
                    last_published_motion_key=last_published_motion_key,
                )
                if candidate.candidate is None:
                    high = scale
                    failed_scale = scale
                else:
                    low = scale
                    best = candidate

            if best.candidate is None:
                return MotionFeasibilityResult(
                    candidate=None,
                    body_pose_infeasible=True,
                    failed_scale=failed_scale,
                )

            self._restore_accepted_state(best)
            return MotionFeasibilityResult(
                candidate=best.candidate,
                failed_scale=failed_scale,
            )

        neutral_body = self._try_candidate(
            previous_state,
            stride_direction,
            rotation_direction,
            body_translation,
            body_rotation,
            walking_scale=0.0,
            body_scale=0.0,
            last_published_motion_key=last_published_motion_key,
        )
        if neutral_body.redundant:
            return neutral_body
        if neutral_body.candidate is None:
            return MotionFeasibilityResult(
                candidate=None,
                body_pose_infeasible=True,
                failed_scale=0.0,
            )

        low = 0.0
        high = 1.0
        best = neutral_body
        failed_scale = zero_motion.failed_scale
        for _ in range(self._search_iterations):
            body_scale = (low + high) / 2.0
            candidate = self._try_candidate(
                previous_state,
                stride_direction,
                rotation_direction,
                body_translation,
                body_rotation,
                walking_scale=0.0,
                body_scale=body_scale,
                last_published_motion_key=last_published_motion_key,
            )
            if candidate.candidate is None:
                high = body_scale
                failed_scale = body_scale
            else:
                low = body_scale
                best = candidate

        if best.candidate is None:
            return MotionFeasibilityResult(
                candidate=None,
                body_pose_infeasible=True,
                failed_scale=failed_scale,
            )

        self._restore_accepted_state(best)
        return MotionFeasibilityResult(
            candidate=best.candidate,
            failed_scale=failed_scale,
        )

    def _try_candidate(
        self,
        previous_state: dict,
        stride_direction: Point3D,
        rotation_direction: float,
        body_translation: Point3D,
        body_rotation: Point3D,
        walking_scale: float,
        body_scale: float,
        last_published_motion_key: tuple | None,
    ) -> MotionFeasibilityResult:
        self._restore_motion_state(previous_state)
        scaled_stride = _scale_planar_stride(stride_direction, walking_scale)
        scaled_rotation = rotation_direction * walking_scale
        scaled_body_translation = _scale_point(body_translation, body_scale)
        scaled_body_rotation = _scale_point(body_rotation, body_scale)
        try:
            feet_target_window = self._build_feet_target_window(
                scaled_stride,
                scaled_rotation,
                scaled_body_translation,
                scaled_body_rotation,
            )
            motion_key = self._motion_window_key(feet_target_window)
            motion_state = self._snapshot_motion_state()

            if motion_key == last_published_motion_key:
                return MotionFeasibilityResult(candidate=None, redundant=True)

            trajectory_targets = self._solve_trajectory_targets(feet_target_window)
        finally:
            self._restore_motion_state(previous_state)

        if trajectory_targets is None:
            return MotionFeasibilityResult(
                candidate=None,
                failed_scale=walking_scale,
            )

        return MotionFeasibilityResult(
            candidate=FeasibleMotionCandidate(
                scale=walking_scale,
                body_scale=body_scale,
                stride_direction=scaled_stride,
                rotation_direction=scaled_rotation,
                body_translation=scaled_body_translation,
                body_rotation=scaled_body_rotation,
                feet_target_window=feet_target_window,
                trajectory_targets=trajectory_targets,
                motion_key=motion_key,
                motion_state=motion_state,
            )
        )

    def _restore_accepted_state(self, result: MotionFeasibilityResult) -> None:
        if result.candidate is not None:
            self._restore_motion_state(result.candidate.motion_state)


def _scale_planar_stride(stride_direction: Point3D, scale: float) -> Point3D:
    return Point3D(
        [
            float(stride_direction.x) * scale,
            float(stride_direction.y) * scale,
            float(stride_direction.z),
        ],
        stride_direction.label,
    )


def _scale_point(point: Point3D, scale: float) -> Point3D:
    return Point3D(
        [
            float(point.x) * scale,
            float(point.y) * scale,
            float(point.z) * scale,
        ],
        point.label,
    )
