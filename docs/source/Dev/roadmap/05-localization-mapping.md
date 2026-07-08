# Phase 5 — Localization and mapping in the home

With odometry (Phase 3) and a camera (Phase 4), the robot can now answer "where am I in the
house?" This is the hardest perception phase, and the plan deliberately has a **pragmatic track**
and an **ambitious track** — the pet should start navigating with the simple one.

## The constraint that shapes everything

Dr.QP has a *single fixed monocular camera* and a Raspberry Pi. Classic dense visual SLAM
(RTAB-Map with RGB-D, stereo VIO) is off the table onboard. Monocular SLAM has unobservable scale
and struggles with a bouncing, low viewpoint. So:

- **Scale comes from leg odometry** (Phase 3), not from vision.
- **Heavy vision may run off-board** on a home server; DDS over Wi-Fi makes a remote node
  identical to a local one. Safety never depends on the off-board link.

## Pragmatic track (recommended first): AprilTag anchors + odometry

Print a handful of AprilTags and place them at fixed spots (door frames, furniture legs, the
charging dock). Then:

1. `apriltag_ros` detects tags in the camera stream (cheap enough for the Pi at a few Hz).
2. A small landmark localizer (or `fuse`/`robot_localization` with pose updates) corrects the
   drifting odometry whenever a known tag is seen → publishes `map → odom`.
3. The "map" is a hand-made YAML of tag poses + a floor plan occupancy grid (drawn once, or traced
   from a phone LiDAR scan) for Nav2.

This gives *reliable, restartable* global localization in a real home for a weekend of work, and
it never fights fur-kicked rugs or lighting changes. The charging dock gets a tag by definition —
docking becomes trivial later.

Deliverables: tag map format + loader, landmark correction node, `map → odom` TF, rviz view of
robot-in-floor-plan, sim world with textured tags for CI tests.

## Ambitious track: visual(-inertial) SLAM

Once the pragmatic track works, upgrade the same interfaces:

- **Candidates**: RTAB-Map in monocular+odometry mode, or an external VIO (OpenVINS, ORB-SLAM3
  mono-inertial) feeding RTAB-Map for mapping. All publish the same `map → odom` TF the tag
  localizer owned — downstream (Nav2) does not change.
- **Depth for mapping**: monocular depth networks (e.g. Depth-Anything-class models) running
  off-board or on an accelerator (Hailo/Coral) can synthesize an RGB-D stream for RTAB-Map's
  occupancy mapping. Treat this as experimental; validate against the hand-drawn floor plan.
- IMU + leg odometry remain the motion backbone; vision corrects, never drives.

## Map lifecycle for a home pet

- Maps persist across reboots (`map_server` YAML + image for the grid; tag YAML for anchors).
- Rooms get *names* ("kitchen", "dock") in a semantic YAML — Phase 7's LLM will ground
  "go to the kitchen" through it.
- Re-mapping is an explicit maintenance action, not a background process, until the ambitious
  track is trusted.

## Milestones

1. Sim world of a small apartment with AprilTags; bridge camera; CI test: robot spawns, sees a
   tag, `map → odom` converges to ground truth within tolerance.
2. Tag map YAML format + landmark localizer node.
3. Hardware: tags in one real room, floor plan grid, robot localizes and re-localizes after
   being carried ("kidnapped robot" test).
4. Semantic locations YAML + a `robot.where_am_i` MCP tool.
5. (Ambitious) RTAB-Map/VIO experiment behind the same TF contract, evaluated against tag
   localization.

## Risks and notes

- Wi-Fi dropouts: `map → odom` freezes, odometry keeps the robot sane; Nav2 must tolerate stale
  global pose (it does — it just degrades).
- A 15 cm viewpoint sees chair legs, not chairs. The floor-plan grid plus generous costmap
  inflation covers what the camera cannot.
- Kidnapped-robot recovery with tags is trivial (see any tag); with pure VIO it is not — another
  reason tags stay even after SLAM works.

## Definition of done

- Robot knows its pose in a named-room map of at least one real room, survives restarts and
  kidnapping, with CI-tested sim equivalent.

Next: {doc}`06-navigation`.
