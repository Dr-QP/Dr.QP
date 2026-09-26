---
type: codebase
description: The drqp_imu node — reads a BNO055 over Raspberry Pi I2C and publishes standard Imu, MagneticField and Temperature messages.
source: packages/runtime/drqp_brain/drqp_brain/imu_node.py
source_digest: sha256:d9f7caeffc2743a7c6ade4b8317670168faa5fa9f5779139e07382014dc6637a
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_brain/drqp_brain/imu_node.py
---

# imu_node

The on-robot IMU driver. `bringup.launch.py` starts it only when
`use_gazebo=false` and `load_imu=true`. In simulation, Gazebo bridges
`/imu/data` in its place.

## Public surface

- The `drqp_imu` executable (`ImuNode`).
- Parameters:
  - `frame_id` (`drqp/imu_link`)
  - `publish_rate_hz` (100.0; it must be > 0)
  - `i2c_address` (`0x28`)
  - `publish_temperature` (true)
- Publishes:
  - `/imu/data` (`sensor_msgs/Imu`)
  - `/imu/mag` (`MagneticField`, in tesla)
  - `/imu/temperature`
- `Bno055Sensor`: the hardware adapter, with `read_sample()` returning an
  `ImuSample`.

## How it works

A timer reads one sample per period through the Adafruit `adafruit_bno055`
library (Blinka `board`/`busio`). A sample without gyro or accelerometer data is
skipped with a warning. A missing quaternion sets
`orientation_covariance[0] = -1`, which the brain treats as "tilt unavailable".
The magnetometer is converted from µT to T.

## Depends on

- `adafruit_bno055` and Blinka, which are imported lazily inside `Bno055Sensor`.
- `rclpy`, `sensor_msgs`.

## Invariants & gotchas

- If sensor setup fails, the node is destroyed and `SensorInitializationError`
  is raised with a hint to run on hardware with I2C enabled. Off the Pi, the
  node cannot start.
- The gyro and accelerometer covariance marker is misused (see
  [Bug: IMU covariance marks gyro and accelerometer data as unavailable](../../../../bugs/imu-covariance-marks-data-unavailable.md)).
- `frame_id` must match the URDF `imu_link`. The brain's tilt math uses the
  fixed mount rotation, not TF (see
  [balance_controller](balance_controller.md)).

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:99` — `Bno055Sensor`
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:124` — `ImuNode`
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:131` — `publish_rate_hz`
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:145` — `/imu/data`
  publisher
- `packages/runtime/drqp_brain/drqp_brain/imu_node.py:182` — missing orientation
  marker
