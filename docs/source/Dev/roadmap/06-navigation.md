# Phase 6 — Autonomous navigation with Nav2

Everything so far was scaffolding for this moment: the robot walks to a goal by itself. Thanks to
Phases 2–5, Dr.QP now looks to Nav2 exactly like a small differential/omni base: it executes
`cmd_vel`, publishes `odom → base_link`, has `map → odom`, and a static map. Nav2 is designed for
precisely that contract.

## What Nav2 gives us vs. what we configure

Nav2 brings the planner, controller, costmaps, recovery behaviors, and behavior trees. Our job is
configuration honest to a small hexapod:

- **Footprint**: ~0.35 × 0.30 m rectangle (legs out). Use the real polygon, not a radius — door
  thresholds and chair-leg forests are the whole game indoors.
- **Kinematics**: hexapods are **omnidirectional** — use a controller that exploits lateral
  stride (`nav2` Rotation Shim + MPPI or DWB with y-velocity enabled). Max speeds come from the
  Phase 2 measurements; keep accelerations gentle (gait transitions are not instant).
- **Costmaps**:
  - Global: static floor-plan/tag map (Phase 5) + inflation.
  - Local: this is the thin spot — with no depth sensor, start with the static map + generous
    inflation and _slow speeds_. Add sensing incrementally (below).
- **Recoveries**: back up, rotate in place (both natural for a hexapod), and a custom "raise
  body & re-settle" behavior for leg snags.

## Local obstacle sensing options (in order of effort)

1. **None (static map only)** — honest start; the home is mostly static, the pet is light and slow.
2. **Monocular floor-plane obstacle detection** — segment "not floor" in the camera image
   (classical color/texture or a tiny segmentation net off-board), project onto the ground plane,
   feed as a `PointCloud2`/costmap layer. Surprisingly effective at pet height.
3. **Proximity sensing** — one or two forward VL53L5CX time-of-flight zones are cheap, tiny, and
   give real 3D obstacle returns for the local costmap. Recommended before ambitious vision.
4. **Contact as last resort** — Phase 8 feet sensors + IMU disturbance detection as a "bumper"
   layer.

## Behavior layer

Nav2's behavior trees orchestrate a single navigation run. Above them sits the _pet_ logic
(Phase 7), which decides **where** to go. This phase ships:

- `NavigateToPose` + `NavigateThroughPoses` actions configured and tested.
- Named-location goals: `go_to("kitchen")` resolves through the semantic YAML from Phase 5.
- MCP tools `robot.go_to`, `robot.cancel_goal`, `robot.nav_status` in `drqp_robot_mcp` — making
  the robot navigable by AI agents and remote humans alike.
- Safety envelope: navigation is only accepted in `torque_on`; kill switch and joystick (via the
  Phase 2 mux) always preempt; Nav2 velocity output clamped by the watchdogged `cmd_vel` path.

## Milestones

1. Nav2 bringup in sim apartment world: goal pose in rviz → robot walks there. CI launch test:
   reach a goal 3 m away through a doorway within time and position tolerance.
2. Omnidirectional tuning: doorway alignment uses lateral stepping (verify in test).
3. Recovery behaviors, including the custom re-settle.
4. Named locations + MCP tools.
5. Hardware: one-room → two-room navigation with tags; measure success rate over ≥20 runs.
6. Optional: ToF sensor integration as a local costmap layer.

## Risks and notes

- **Controller ↔ gait impedance mismatch** is the classic legged-robot Nav2 pain: controllers
  that demand instant velocity reversals make gaits stumble. MPPI with tuned constraints, low
  accel limits, and the Phase 2 watchdog smoothing handle it.
- Carpets change effective velocity — Phase 3 odometry closes the loop, Nav2 replans; expect it.
- Keep planning off the Pi if CPU is tight (Nav2 runs fine off-board; only `cmd_vel` consumption
  is safety-local).

## Definition of done

- "Send the robot to the kitchen" works from rviz, CLI, and MCP, in sim (CI) and in the real
  home, with graceful failure and recovery.

Next: {doc}`07-interaction` — which can largely be built in parallel.
