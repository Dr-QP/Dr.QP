---
id: RM-03
title: Leg odometry + IMU fusion (odom → base_link)
status: proposed
depends_on: [RM-02]
packages: [drqp_brain, drqp_kinematics, drqp_interfaces, drqp_gazebo]
---

# RM-03 — State estimation

## Objective

Robot-generated odometry: leg (kinematic) odometry fused with IMU via `robot_localization` EKF,
publishing `/odometry/filtered` and TF `odom → drqp/base_link` in sim and on hardware.

## Interfaces

- NEW msg `drqp_interfaces/GaitPhase`: `float32 phase`, `string gait_type`,
  `bool[6] leg_in_stance` (leg order documented; match `HexapodModel.legs`). Published by walk
  controller each tick on `/gait/phase`. `leg_in_stance` here is the **gait-phase** stance source;
  RM-08 adds a **sensed** source (`FootContacts`) selectable via `stance_source`. Keep them one
  interface — see [Program relationships](../README.md#stance-detection-has-three-sources-keep-them-one-interface).
- NEW node `leg_odometry` (`drqp_brain`): subscribes `/joint_states`, `/gait/phase`; publishes
  `nav_msgs/Odometry` on `/odom_legs` (frame `odom`, child `drqp/base_link`, twist + integrated
  pose, covariances: xy/yaw moderate, z/roll/pitch large).
- `robot_localization` `ekf_node`: fuses `/odom_legs` (vx, vy, vyaw) + `/imu/data`
  (roll, pitch, yaw-differential, yaw rate). Owns `odom → drqp/base_link`. Config
  `drqp_brain/config/ekf.yaml`; added to `bringup.launch.py` and sim launch.
- **Rename** sim ground truth bridge topic `/odom` → `/odom_ground_truth` in
  `drqp_gazebo_bridge.yml`; update all tests referencing it.

## Algorithm (leg_odometry)

Per `/joint_states` sample: FK via `drqp_kinematics` for stance feet; remove commanded body sway
by expressing foot positions in the _neutral body frame_ (undo `hexapod.body_transform` — requires
brain to publish it or recompute from `MovementCommand`; simplest: include body transform in
`GaitPhase` as `geometry_msgs/Transform body_transform`). Rigid 2D least-squares (x, y, yaw) of
stance-foot displacement between samples ⇒ body twist; integrate for pose. Feet entering/leaving
stance are excluded that tick.

## Acceptance criteria

- [ ] Unit tests with synthetic FK walks: straight line, pure rotation, lateral — recovered
      twist within 5 % (no noise).
- [ ] Launch test: scripted 2 m out-and-back + 360° spin in sim; `/odometry/filtered` vs
      `/odom_ground_truth`: translation drift < 10 % of distance, heading error < 15°.
- [ ] `view_frames`-equivalent test: `odom → drqp/base_link` present, published ≥ 8 Hz.
- [ ] Body-pose commands (translation/rotation without stepping) produce near-zero odometry
      displacement (test).
- [ ] Existing balance/movement tests green with EKF running.

## Constraints

- `map → odom` reserved for RM-05; nothing here publishes it.
- No consumer may subscribe to `/odom_ground_truth` at runtime (evaluation only).
- BNO055 yaw fused differentially only (indoor magnetometer untrusted).
