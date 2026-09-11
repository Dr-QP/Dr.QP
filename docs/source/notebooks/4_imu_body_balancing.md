---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.17.2
kernelspec:
  display_name: .venv
  language: python
  name: python3
---

# 4. IMU-Based Stationary Body Posture

This notebook documents the stationary posture loop exposed through
`/robot/balance_mode`. When the mode is enabled, the controller stops the gait,
captures the current IMU-derived body tilt as its target, and holds that target
with all six feet at their stationary stance targets. This is an attitude-only
posture controller; it has no contact estimation or support-polygon awareness
and must not be used for walking while balancing.

## Control idea

The balancing loop derives body attitude from the orientation reported on
`/imu/data`. The IMU chip is mounted on the body at a fixed, known rotation
(the `base_center_to_imu` joint in `body.urdf.xacro`), so `body_tilt_from_imu`
de-rotates that fixed mount rotation from the reported orientation to recover
body roll/pitch, instead of looking up an IMU-to-base transform at runtime.

The robot model applies body rotation through the inverse-kinematics body
transform, so posture hold is implemented as a roll/pitch correction from the
captured target while operator body rotation remains suppressed:

```{code-cell} ipython3
from drqp_brain.balance_controller import apply_imu_balance
from drqp_kinematics.geometry import Point3D

requested_body_rotation = Point3D([0.0, 0.0, 0.0])
measured_body_tilt = Point3D([0.08, -0.05, 0.0])
target_body_tilt = Point3D([0.02, -0.01, 0.0])

apply_imu_balance(
    requested_body_rotation,
    measured_body_tilt,
    target_body_tilt=target_body_tilt,
)
```

The compensation corrects only roll and pitch relative to the captured target.
Omitting `target_body_tilt` uses a level-to-world target; the runtime always uses
the captured target while stationary posture mode is active.

## Safety contract and validated envelope

- The robot must be armed (`torque_on`) and `/imu/data` must be fresh when the
  mode is enabled. Unavailable or stale IMU data disables posture hold and
  leaves semantic movement cleared.
- Enabling the mode discards the active stride, yaw, body-translation, and user
  body-rotation command and resets gait phase. Commands received while active
  are ignored. Disabling the mode cannot replay them; walking requires a fresh
  movement command.
- Before solving, the complete correction is scaled by one scalar against
  unclamped analytic IK for all six stationary foot targets. Workspace and URDF
  joint limits are checked first; the normal MoveIt state validator still checks
  the resulting complete state for self-collision.
- For the default `0°, -35°, 130°` stance and production URDF limits, the
  hardware-safe correction envelope (including margin) is **±0.10 rad on either
  pure roll or pure pitch axis**, or **±0.06 rad per axis for simultaneous roll
  and pitch**. These are body-correction angles, not a promise about terrain
  slope or static stability.
- Requests outside that envelope are reduced automatically for the current
  stance. Saturation emits the throttled `drqp_brain` diagnostic
  `stationary_balance_correction_saturated:<constrained legs>` with the applied
  scale.

## Extracting body tilt

`body_tilt_from_imu` recovers roll and pitch from the raw `/imu/data`
orientation by de-rotating the fixed IMU mount rotation
(`BASE_CENTER_TO_IMU_ROTATION`): since the IMU is rigidly mounted on the body,
its reported orientation is `imu = body * mount`, so `body = imu * mount.inv()`.
Yaw is dropped because balancing only corrects roll and pitch.

`body_tilt_from_imu` only reads the `x`, `y`, `z`, `w` fields of the
orientation, so this notebook uses a plain stand-in for
`geometry_msgs.msg.Quaternion` instead of the real ROS message type, since
this environment does not have ROS installed:

```{code-cell} ipython3
from dataclasses import dataclass

from drqp_brain.balance_controller import BASE_CENTER_TO_IMU_ROTATION, body_tilt_from_imu
from scipy.spatial.transform import Rotation as R


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float


body_in_world = R.from_euler('xyz', [0.10, -0.06, 0.25], degrees=False)
imu_in_world = body_in_world * BASE_CENTER_TO_IMU_ROTATION
qx, qy, qz, qw = imu_in_world.as_quat()

body_tilt_from_imu(Quaternion(x=qx, y=qy, z=qz, w=qw))
```

## Tuning notes

- `imu_balance_gain` and `imu_balance_max_tilt_rad` bound the requested
  correction, but the six-leg reachability check is the final authority.
- The validated envelope assumes the documented default stance. A translated
  body or a different stance can have less margin, so the runtime recomputes the
  scalar bound from the current stationary targets every tick.
- This mode is fail-safe posture hold, not dynamic stabilization. Hardware use
  still requires conservative speed, a clear area, and an operator-ready torque
  cutoff.
