---
id: RM-04
title: Camera bring-up (hardware + simulation)
status: proposed
depends_on: [RM-01]
packages: [drqp_control, drqp_gazebo, drqp_robot_mcp]
---

# RM-04 — Camera

## Objective

`sensor_msgs/Image` + `CameraInfo` on identical topics in sim and hardware, correct TF on the
existing `drqp/camera` link, stored calibration, and an MCP snapshot tool.

## Interfaces

- Topics: `/camera/image_raw`, `/camera/camera_info` (frame_id `drqp/camera`). Compressed
  transport enabled for off-robot viewing.
- Sim: gz-sim `camera` sensor in `urdf/gazebo.xacro` attached to `drqp/camera`
  (640×480 @ 15 fps, ~90° HFOV to match Pi Cam 3 Wide); bridge entries in
  `drqp_gazebo_bridge.yml` (`gz.msgs.Image → sensor_msgs/msg/Image`,
  `gz.msgs.CameraInfo → sensor_msgs/msg/CameraInfo`).
- Hardware: `camera_ros` (libcamera) node in hardware bringup launch, params
  (resolution/fps/format) in `drqp_control/config/camera.yaml`; calibration YAML loaded so
  `camera_info` is populated.
- MCP: `robot.camera.snapshot` tool in `drqp_robot_mcp/server.py` returning a JPEG (base64 or
  file path) captured from `/camera/image_raw`.

## Tasks

1. URDF: verify/adjust `base_bottom_to_camera` origin/RPY against physical mount; consider
   10–15° downward pitch (single source of truth in xacro; sim and TF inherit).
2. Sim sensor + bridge + rviz config update.
3. Hardware driver bring-up + intrinsic calibration (`camera_calibration` checkerboard);
   store YAML in `drqp_control/config/`.
4. MCP snapshot tool + test (sim).

## Acceptance criteria

- [ ] Launch test: after sim bringup, ≥ N images arrive in T seconds; `CameraInfo` has nonzero
      intrinsics; TF `drqp/base_link → drqp/camera` resolves; image frame_id matches TF frame.
- [ ] `robot.camera.snapshot` returns a decodable JPEG in sim test.
- [ ] Hardware: same topics visible; calibration reprojection error < 0.5 px reported.
- [ ] CPU cost on Pi measured and recorded vs RM-01 baseline.

## Constraints

- Topic names identical between sim and hardware (consumers must not care).
- Raw image stays on-robot; only compressed leaves over Wi-Fi (documented QoS/transport).
