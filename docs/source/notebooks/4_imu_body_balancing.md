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

# 4. IMU-Based Body Balancing

This notebook documents the balancing loop added to the walking controller.
When balance mode is enabled, the controller captures the current IMU-derived
body tilt as the target orientation and holds that target while the feet stay in
contact with sloped terrain.

## Control idea

The balancing loop uses the orientation reported on `/imu/data` directly as the
robot body attitude. IMU publishers are responsible for any sensor-to-body
conversion before publishing, so `drqp_brain` no longer compensates for IMU
mount rotation or looks up an IMU-to-base transform.

The robot model already applies body rotation through the inverse-kinematics
body transform, so balancing is implemented as a small correction added to the
requested body rotation before each walking step:

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

The compensation keeps yaw commands untouched and only corrects roll and pitch
relative to the captured target. Omitting `target_body_tilt` uses a level-to-world
target.

## Extracting body tilt

`body_tilt_from_imu` extracts roll and pitch directly from the `/imu/data`
orientation, since that orientation is already expressed in the robot body
frame. Yaw is dropped because balancing only corrects roll and pitch.

`body_tilt_from_imu` only reads the `x`, `y`, `z`, `w` fields of the
orientation, so this notebook uses a plain stand-in for
`geometry_msgs.msg.Quaternion` instead of the real ROS message type, since
this environment does not have ROS installed:

```{code-cell} ipython3
from dataclasses import dataclass

from drqp_brain.balance_controller import body_tilt_from_imu
from scipy.spatial.transform import Rotation as R


@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float


body_in_world = R.from_euler('xyz', [0.10, -0.06, 0.25], degrees=False)
qx, qy, qz, qw = body_in_world.as_quat()

body_tilt_from_imu(Quaternion(x=qx, y=qy, z=qz, w=qw))
```

## Tuning notes

- Roll and pitch compensation are applied relative to the captured target tilt
  and clamped before they are applied to the body transform.
- Yaw is left under operator control.
- If IMU data becomes unavailable or stale, the walking controller falls back to
  the requested body rotation without balance compensation.
