# Phase 3 — State estimation: leg odometry + IMU fusion

A robot that navigates needs a continuous answer to "how have I moved since I started?" — the
`odom → base_link` transform. Wheeled robots get it from encoders; Dr.QP gets it from its legs.
This phase builds **leg (kinematic) odometry**, fuses it with the IMU, and publishes the standard
TF and `/odom` topic _from the robot itself_ (today's `/odom` exists only in sim, as Gazebo ground
truth).

## How leg odometry works on a hexapod

At every control tick the robot knows, from forward kinematics of `/joint_states`, where each foot
is relative to the body. Feet that are **in stance** (on the ground) are stationary in the world,
so if the stance feet moved backwards 6 mm relative to the body, the body moved forwards 6 mm in
the world. Averaging over all stance feet (least-squares rigid transform) gives body translation
_and_ yaw per tick.

Until feet sensors exist (Phase 8), stance detection comes from the gait generator itself — the
`ParametricGaitGenerator` knows exactly which legs are in stance at the current phase. That is an
approximation (slip, early/late touchdown), which is why the IMU fusion matters.

```text
/joint_states ─▶ FK (drqp_kinematics) ─▶ stance-feet rigid motion ─▶ twist + integrated pose
                                              ▲                              │
gait phase (which legs are stance) ───────────┘                              ▼
                                                              /odom_legs (nav_msgs/Odometry)
/imu/data (orientation, angular vel, accel) ──▶ robot_localization EKF ──▶ /odometry/filtered
                                                                        └─▶ TF odom → base_link
```

## Design decisions

1. **New node `leg_odometry`** in `drqp_brain` (Python first; port to C++ only if profiling says
   so). Inputs: `/joint_states` + gait phase (published by the walk controller as a small custom
   message, or recomputed). Output: `nav_msgs/Odometry` on `/odom_legs` with a realistic
   covariance (large on z, moderate on x/y/yaw).
2. **Fusion via `robot_localization`** — the standard EKF node, not hand-rolled. Config:
   fuse `/odom_legs` (x, y, yaw velocities) + `/imu/data` (orientation, yaw rate). It owns the
   `odom → base_link` TF.
3. **Frame contract**: `odom → drqp/base_link` from the EKF; `map → odom` reserved for Phase 5.
   The Gazebo ground-truth odometry moves to `/odom_ground_truth` — it becomes the _evaluation_
   signal, never a runtime input.
4. **Body-pose subtlety**: the brain deliberately shifts the body relative to the feet (body
   translation/rotation commands, balance mode). Leg odometry must measure _locomotion_, not body
   sway — compute foot motion in the _neutral body frame_ by removing the commanded
   `body_transform` (the hexapod model already tracks it).

## Milestones

1. Gait-phase/stance publication from the walk controller.
2. `leg_odometry` node with unit tests against synthetic walks (known stride → known odometry).
3. `robot_localization` EKF in bringup; TF `odom → base_link` live in sim and rviz.
4. **Drift benchmark** (launch test): walk a scripted 2 m out-and-back + 360° turn in sim; report
   translational drift as % of distance and heading error vs `/odom_ground_truth`. Gate CI on a
   generous threshold (e.g. <10 % translation, <15° heading), record the trend in docs.
5. Hardware sanity check: walk a measured 2 m line, compare integrated odometry.

## Risks and notes

- **Slip on smooth floors** is the dominant error source; that is expected. SLAM (Phase 5)
  corrects drift globally — odometry only needs to be _locally_ smooth and roughly scaled.
- The BNO055 provides fused orientation onboard; trust its yaw only relatively (magnetometer
  indoors is unreliable) — configure the EKF for `differential` yaw from IMU.
- Covariance tuning is where this phase's real time goes. Budget for it.
- Keep the 8 Hz tick in mind: odometry integrates at brain rate; that's adequate for walking
  speeds (<0.2 m/s ⇒ <2.5 cm per tick).

## Definition of done

- `ros2 run tf2_tools view_frames` shows `odom → drqp/base_link` on sim and hardware.
- Drift benchmark in CI with recorded numbers.
- Balance mode and all existing tests still pass.

Next: {doc}`04-camera` (independent — can run in parallel with this phase).
