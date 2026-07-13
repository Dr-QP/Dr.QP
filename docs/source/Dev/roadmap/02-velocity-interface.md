# Phase 2 — Metric velocity interface (`cmd_vel`)

The single highest-leverage refactor in the whole roadmap. Today the robot understands
"stride 60 % forward"; the entire ROS navigation ecosystem speaks "0.12 m/s forward,
0.3 rad/s yaw". Bridging that gap turns Dr.QP from a bespoke toy into a standard mobile base that
Nav2, teleop tools, and future RL policies can all drive.

## Goal

Accept `geometry_msgs/Twist` on `/cmd_vel` and walk at _that actual metric velocity_ (within
tolerance), while keeping the existing joystick/semantic layer fully working.

## Why this must come before navigation

Nav2's controllers emit `cmd_vel` and _assume the base executes it_: local planning, obstacle
avoidance timing, and goal checking all rely on commanded ≈ executed velocity. A hexapod gait has
a well-defined body velocity — stride length × step frequency for the active gait — so the mapping
is derivable, then verifiable against ground truth in Gazebo.

## Design

```text
                      ┌───────────────────────────── semantic layer (kept) ─────────────┐
 /joy ─▶ translator ─▶│ /robot/movement_command (gait, body pose, normalized overrides) │
                      └──────────────────────────────┬──────────────────────────────────┘
                                                     ▼
 /cmd_vel (Twist) ────────────────────────▶ locomotion controller (drqp_brain)
                                                     │  velocity ⇄ stride mapping
                                                     ▼
                                     WalkController → IK → joint trajectories
```

1. **Characterize the gait velocity model.** For each gait, body velocity is
   `v = stride_length × cycle_frequency × duty_factor_term`. Rather than deriving it purely on
   paper, measure it: drive the sim across the stride/frequency envelope and fit
   stride ↦ velocity per gait. The existing `/odom` ground truth bridge and the
   `robot.recording.*` MCP tools are exactly the instrumentation needed.
2. **Invert the model in the brain.** A new `/cmd_vel` subscriber converts requested
   `(v_x, v_y, ω_z)` into stride direction/length and phase rate, saturating at the robot's
   measured maximum. Publish the actually-achievable command
   back (standard practice: `/cmd_vel` in, best-effort execution).
3. **Command arbitration.** Joystick and `/cmd_vel` must not fight. A simple priority mux
   (joystick > autonomy, with timeout-based release) is enough; `twist_mux` from the ROS ecosystem
   does this out of the box.
4. **Safety semantics.** Zero/stale `cmd_vel` (watchdog ~0.5 s) → smooth stop. Kill switch and
   state machine keep overriding everything, unchanged.

## Milestones

1. Velocity characterization notebook + fitted per-gait model checked into
   `docs/source/notebooks/` (there is a notebook culture here already — use it).
2. `/cmd_vel` subscriber in `drqp_brain` with metric mapping and watchdog.
3. `twist_mux` (or equivalent) in bringup; joystick path routed through it.
4. Sim launch test: command 0.1 m/s forward for 10 s → ground-truth displacement within ±20 %
   (tighten with feedback in Phase 3); same for lateral, yaw, and combined motion.
5. Hardware validation: tape-measure test at three speeds, results recorded in docs.

## Risks and notes

- **Slip and load change the model** — hardware will be slower than sim. Ship with generous
  tolerance; Phase 3 odometry enables closed-loop correction later.
- **Gait switching under velocity control**: keep gait selection semantic (auto-select by speed
  can come later — wave for precision, tripod for speed).
- The brain's 8 Hz loop is fine for this phase; velocity fidelity comes from the model, not loop
  rate.

## Definition of done

- Nav2's `teleop_twist_keyboard` drives the robot in sim and on hardware with no Dr.QP-specific
  tooling.
- Velocity accuracy tests pass in CI.
- Joystick behavior is unchanged for a human operator.

Next: {doc}`03-state-estimation`.
