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
The goal is to keep the robot body flat in the world XY plane while the feet
stay in contact with sloped terrain.

## Control idea

The balancing loop uses the orientation reported on `/imu/data` and converts it
from `drqp/imu_link` into `drqp/base_center_link`.

The robot model already applies body rotation through the inverse-kinematics
body transform, so balancing is implemented as a small correction added to the
requested body rotation before each walking step:

```{code-cell} ipython3
from drqp_brain.balance_controller import apply_imu_balance
from drqp_brain.geometry import Point3D

requested_body_rotation = Point3D([0.0, 0.0, 0.0])
measured_body_tilt = Point3D([0.08, -0.05, 0.0])

apply_imu_balance(requested_body_rotation, measured_body_tilt)
```

The compensation keeps yaw commands untouched and only cancels roll and pitch.

## IMU frame conversion

The IMU is mounted on a fixed joint relative to `drqp/base_center_link`, so the
measured orientation must be rotated back into the body frame before extracting
roll and pitch.

```{code-cell} ipython3
from drqp_brain.balance_controller import (
    BASE_CENTER_TO_IMU_ROTATION,
    body_tilt_from_imu,
)
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

base_in_world = R.from_euler('xyz', [0.10, -0.06, 0.25], degrees=False)
imu_in_world = base_in_world * BASE_CENTER_TO_IMU_ROTATION
qx, qy, qz, qw = imu_in_world.as_quat()

body_tilt_from_imu(Quaternion(x=qx, y=qy, z=qz, w=qw))
```

## Tuning notes

- Roll and pitch compensation are clamped before they are applied to the body
  transform.
- Yaw is left under operator control.
- If IMU data becomes unavailable or stale, the walking controller falls back to
  the requested body rotation without balance compensation.
