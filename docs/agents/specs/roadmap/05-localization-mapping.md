---
id: RM-05
title: Home localization and mapping (AprilTag anchors first)
status: proposed
depends_on: [RM-03, RM-04]
packages: [drqp_gazebo, 'NEW: drqp_localization']
---

# RM-05 — Localization and mapping

## Objective

Global pose in a persistent home map: `map → odom` TF from AprilTag landmark corrections over the
RM-03 odometry, plus a named-location semantic layer. Visual SLAM is a follow-up experiment
behind the same TF contract, not part of this spec's acceptance.

## Interfaces

- NEW package `drqp_localization` (Python):
  - `apriltag_ros` (or `apriltag` python binding) detection on `/camera/image_raw` →
    `/tag_detections`.
  - `landmark_localizer` node: consumes `/tag_detections` + tag map + `/odometry/filtered`;
    publishes `map → odom` TF and `/pose_in_map` (`geometry_msgs/PoseWithCovarianceStamped`).
    Correction strategy: on tag sighting, solve map-pose from tag pose ⊕ camera extrinsics;
    smooth `map → odom` update (no jumps > configurable rate while moving).
  - Map assets in `config/`: `tag_map.yaml` (tag id → pose in `map`, size), `locations.yaml`
    (name → pose, e.g. kitchen, dock), occupancy grid `home.yaml` + `.pgm` served by
    `nav2_map_server` (used by RM-06).
- MCP: `robot.where_am_i` → `{map_pose, nearest_location, confidence}`.

## Sim assets

- New world `apartment.sdf` in `drqp_gazebo/worlds/`: 2 rooms + doorway, ≥ 4 AprilTag models
  (textured planes) at poses matching a checked-in `tag_map.yaml`; camera bridge active.

## Acceptance criteria

- [ ] Launch test: spawn in apartment world facing a tag ⇒ `map → odom` published;
      `/pose_in_map` within 0.15 m / 10° of ground truth after convergence.
- [ ] Kidnapped-robot test: teleport model, walk until tag visible ⇒ pose recovers to tolerance.
- [ ] Drift-correction test: scripted loop between two tags ⇒ pose error bounded (does not grow
      with laps).
- [ ] `locations.yaml` loader with schema validation + unit tests; `robot.where_am_i` works in sim.
- [ ] Restart persistence: same map/tag config reloads; pose re-acquired without re-mapping.

## Constraints

- Only `landmark_localizer` publishes `map → odom`. Never publish `map → base_link` directly.
- Localization must degrade gracefully to odometry-only (stale `map → odom` is legal); consumers
  read the TF timestamp.
- Tag detection budget: ≤ 5 Hz on Pi-class CPU; downscale image if needed.

## Follow-up (separate spec when picked up)

RTAB-Map / VIO experiment replacing the tag localizer behind identical TF + `/pose_in_map`
interfaces; evaluated against tag ground truth.
