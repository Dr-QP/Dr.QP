---
id: RM-10
title: Local obstacle perception and reactive safety
status: proposed
depends_on: [RM-04, RM-06] # RM-04 camera (mono floor-plane source); RM-06 costmap is the consumer
packages: [drqp_gazebo, drqp_control, drqp_navigation, 'NEW: drqp_perception']
---

# RM-10 — Obstacle perception

## Objective

A live obstacle source that populates Nav2's local costmap so navigation reacts to people, pets,
and moved furniture — not just the static home map. This is the follow-up RM-06 leaves as a
**gate on its hardware navigation milestone**: a static-map robot cannot navigate a real home
safely. Source-agnostic (like RM-08's contact interface): sim depth sensor first, then mono
floor-plane detection and/or a real depth/ToF sensor on hardware, all behind one costmap-facing
contract.

## Interfaces

- **Obstacle output (source-agnostic)**: publish obstacles in a costmap-consumable form —
  `sensor_msgs/PointCloud2` (from depth) and/or a projected `sensor_msgs/LaserScan` — on a stable
  frame (`drqp/camera` for vision, a new `drqp/tof_link` for a ToF sensor; declare in URDF).
  Topics `/obstacles/points`, `/obstacles/scan`. Consumers must not care which source produced it.
- **Costmap wiring**: fill the obstacle/voxel + inflation plugin slots RM-06 left configured in
  `nav2_params.yaml` (single source of footprint/inflation with RM-06). Raytrace clearing so
  moved-away obstacles decay out of the costmap.
- **Reactive stop reflex (safety-local, brain)**: NEW fast node/path subscribing the nearest
  obstacle source directly; if any obstacle enters a stop zone in the direction of travel, publish
  a `/robot_event` stop and zero `cmd_vel` — **independent of off-board Nav2** so a network stall
  cannot drive the robot into a wall. Must not toggle the state machine (reuse the idempotent
  de-energize/stop semantics; see RM-07's pick-up-reflex constraint).
- MCP: `robot.obstacle_snapshot` → nearest-obstacle summary (distance/bearing) for the agent.

## Sources

1. **Sim** (`drqp_gazebo`): gz-sim depth camera or `gpu_lidar` sensor in `gazebo.xacro`; bridge
   entries → `PointCloud2`/`LaserScan`. Used for all CI acceptance.
2. **Hardware A — mono floor-plane** (`drqp_perception`, no new hardware): segment the ground
   plane in the RM-04 camera image using the known camera extrinsics/pitch (RM-04) via homography;
   classify non-floor pixels as obstacles; project to `LaserScan`/`PointCloud2`. Cheap; degrades on
   textureless/low-contrast floors (documented).
3. **Hardware B — depth/ToF** (`drqp_control` or `drqp_perception`): a real sensor (depth camera
   or VL53L5CX-class ToF array) → `PointCloud2`. Interface identical to A.

## Sim assets

Dynamic-obstacle world in `drqp_gazebo/worlds/` (or extend `apartment.sdf`): a scripted moving
box and/or a spawnable person/pet model that can cross the robot's path mid-navigation.

## Acceptance criteria

- [ ] Launch test: sim obstacle source publishes `PointCloud2`/`LaserScan` at ≥ 5 Hz; the Nav2
      local costmap shows the obstacle at the correct pose (query costmap or assert path deforms).
- [ ] Reaction test: a dynamic obstacle blocks the planned path mid-`NavigateToPose` ⇒ robot
      replans or stops and does **not** collide (min ground-truth clearance > footprint margin).
- [ ] Clearing test: obstacle removed ⇒ costmap clears within N s and the path re-opens.
- [ ] Floor-plane unit test on recorded frames: known obstacle → non-floor classification within
      tolerance; flat empty floor → no false obstacles above a bounded rate.
- [ ] Reflex test: obstacle entering the stop zone ⇒ stop within one control cycle, with Nav2
      absent/stalled (reflex is not on the Nav2 path).
- [ ] Hardware: real-sensor detection range/latency recorded; **RM-06's hardware navigation
      milestone re-run and now passing** with a live obstacle present.

## Constraints

- The obstacle source (or at minimum the stop reflex) runs **on-robot**; Nav2 may be off-board.
- Detection compute budget on the Pi measured vs the RM-01 baseline; downscale/limit rate to fit.
- No Nav2 forks — costmap plugins + thin perception nodes only.
- Interface identical across sources; no consumer subscribes to a source-specific raw topic.

## Follow-up (separate spec when picked up)

Semantic obstacle classes (person vs pet vs furniture) feeding RM-07 behaviors and social costmap
layers; multi-sensor fusion (vision + ToF) behind the same `/obstacles/*` contract.
