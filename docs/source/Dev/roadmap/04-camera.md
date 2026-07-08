# Phase 4 — Camera bring-up (hardware and simulation)

The camera is the robot's most information-dense sensor and the prerequisite for localization,
navigation, and "seeing" people. The URDF already reserves a `drqp/camera` link on the front of
the body (`body.urdf.xacro`); this phase makes it real — on the Pi and in Gazebo — and calibrated.

## Goal

`sensor_msgs/Image` + `sensor_msgs/CameraInfo` published on the robot and in simulation, with the
same topic names, correct TF, and a stored calibration.

## Hardware

- **Sensor**: Raspberry Pi Camera Module 3 (IMX708) is the current recommendation — wide FoV
  variant preferred for indoor navigation (more features in view, better for a low robot looking
  across a floor). Any libcamera-supported module works.
- **Driver**: [`camera_ros`](https://github.com/christianrauch/camera_ros) (libcamera-based ROS 2
  node). It publishes `image_raw` + `camera_info` and supports compressed transports.
- **Mounting**: fixed, front-facing, per the existing URDF joint (`base_bottom_to_camera`,
  x = 0.132 m). Measure and update the xacro origin/RPY against the real mount; a few degrees of
  unmodeled pitch will visibly corrupt visual odometry later. Consider a slight downward pitch
  (10–15°) so the floor 0.3–2 m ahead is in frame — that is where obstacles live for a 15 cm tall
  robot.

## Simulation

Add a camera sensor to `gazebo.xacro` on the same link (gz-sim `camera` sensor), bridge
`image` and `camera_info` in `drqp_gazebo_bridge.yml`, matching hardware topic names
(`/camera/image_raw`, `/camera/camera_info`). The sim camera uses the *ideal* pinhole intrinsics
from the SDF — good enough for developing every downstream consumer before hardware exists.

## Calibration

1. Intrinsics: `camera_calibration` (checkerboard) once per physical camera; store the YAML in
   `drqp_control/config/` and load it via the driver so `camera_info` is always correct.
2. Extrinsics: verify the URDF camera pose by projecting a known object (e.g. checkerboard on the
   floor at a measured distance) — cheap now, painful to debug inside SLAM later.

## Performance budget (Pi)

- Publish full-rate raw only on-robot (intra-host); ship **compressed** (`image_transport` JPEG,
  or H.264 via `foxglove_bridge`/`web_video_server` for viewing) over Wi-Fi.
- Start at 640×480 @ 15–30 fps. Vision consumers (SLAM, detection) rarely want more from this
  platform, and USB/CSI bandwidth and CPU are precious.
- Measure and record CPU cost against the Phase 1 baseline.

## Milestones

1. Sim camera + bridge + rviz image view; launch test asserting images arrive with plausible
   frame rate and a valid `CameraInfo`/TF chain (`drqp/camera` frame resolves).
2. `camera_ros` bring-up on the Pi, systemd/launch integration in the hardware bringup.
3. Calibration stored + loaded; extrinsics verified.
4. A fun, visible payoff: snapshot MCP tool (`robot.camera.snapshot`) in `drqp_robot_mcp` so an
   agent (or you, from a chat) can see through the robot's eyes. This also seeds Phase 7.

## Risks and notes

- Rolling shutter + a walking hexapod = motion blur and jello. If Phase 5 visual odometry
  struggles, the mitigation order is: shorter exposure, wider FoV, feature-based (not direct)
  SLAM, and only lastly a global-shutter camera swap.
- Keep the camera on a robot-local time base (`header.stamp` from the driver against system
  clock synced via NTP/chrony) — timestamp discipline decides SLAM quality.

## Definition of done

- Same-name image topics in sim and on hardware, with calibration and TF.
- Launch test in CI (sim).
- Snapshot visible in rviz/Foxglove over Wi-Fi and via MCP tool.

Next: {doc}`05-localization-mapping`.
