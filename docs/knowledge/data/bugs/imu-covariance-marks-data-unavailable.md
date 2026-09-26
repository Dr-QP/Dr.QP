---
type: bug
description: drqp_imu sets covariance[0] = -1 on gyro and accelerometer, which sensor_msgs/Imu defines as "no estimate for this field", not "covariance unknown".
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T12:30:00Z
sources:
- id: imu
  resource: packages/runtime/drqp_brain/drqp_brain/imu_node.py
- id: msg
  resource: sensor_msgs/msg/Imu.msg (ROS 2 Jazzy common_interfaces)
---

# Bug: IMU covariance marks gyro and accelerometer data as unavailable

## Symptom

Every `/imu/data` message from the real robot has
`angular_velocity_covariance[0] = -1` and
`linear_acceleration_covariance[0] = -1`, even though both fields carry
measurements. The inline comment says "covariance unknown".

## Reproduction

On the robot, `ros2 topic echo /imu/data --once` shows `-1.0` as the first
element of both covariance arrays.

## Root cause

`sensor_msgs/Imu` uses two different markers:

- An all-zero covariance means "covariance unknown".
- `-1` in element 0 means "no estimate for this data element".

The node uses the second marker where it means the first. Standard consumers
such as `robot_localization` or `imu_filter_madgwick` would ignore the gyro and
accelerometer. The brain reads only `orientation`, so nothing in the repo is
affected today. State estimation (roadmap RM-03) would be.

The orientation marker is correct. The node sets
`orientation_covariance[0] = -1` only when the BNO055 returns no quaternion, and
the brain relies on that to drop balance mode.

## Fix

Leave the gyro and accelerometer covariances at zeros, or fill in diagonal
variances from the BNO055 datasheet. `test_imu_node.py` currently asserts the
`-1` values, so it has to change first (TDD red): assert that element 0 is
non-negative whenever the field is populated.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:194` — gyro covariance set
  to −1
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:202` — accelerometer
  covariance set to −1
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:182` — correct use for a
  missing orientation
- `packages/runtime/drqp_brain/test/test_imu_node.py:219` — test pins the gyro
  value at −1
- `packages/runtime/drqp_brain/drqp_brain/brain_node.py:404` — brain treats a
  negative orientation covariance as unavailable
