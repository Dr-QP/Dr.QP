# Hexapod IK & locomotion: analysis and development guide

In-depth review of the Dr.QP locomotion stack — analytic kinematics, MoveItPy runtime IK,
gait generation, steering, IMU balance, stride-limit calibration, and trajectory execution —
with industry best practices and a phased improvement roadmap.

:::{note}
Reviewed from direct source review of `drqp_kinematics`, `drqp_brain`, and `drqp_moveit`
(July 2026, branch `fix-test-steo-rotation-and-stride`). Line references are to that revision
and will drift over time.
:::

## Executive summary

The locomotion stack is unusually well-engineered for a hobby-scale hexapod: clean layer
separation, an offline reachability calibration (`generate_stride_limits`) that most commercial
hexapod firmwares don't have, whole-state validation with self-collision checking, and a strong
test culture around all of it. The foundations are worth keeping.

:::{important}
**The central architectural tension:** a 3-DOF hexapod leg has a textbook closed-form IK — and
the repo already implements it correctly in `drqp_kinematics` — yet the runtime control loop
solves each leg with MoveIt's _numeric, iterative_ KDL solver through in-process MoveItPy. That
choice buys collision/bounds validation but pays for it with a slow (8 Hz), failure-prone,
binary-outcome hot path that has shaped much of the surrounding code: the 2-point lookahead
workaround, tick-level rollback, dedupe keys, retry-from-home seeding, and the offline
stride-limit table that exists chiefly to keep the numeric solver away from targets it might
reject.

**Recommendation in one line:** make the analytic solver the primary runtime IK with explicit
joint-limit and workspace clamping, keep MoveIt as an offline/CI validation oracle, reformulate
steering as a body twist, and reinvest the freed CPU budget in loop rate, swing-profile quality,
and contact-aware balance.
:::

Top five actions, in order of leverage:

1. **Analytic IK in the hot loop** — deterministic microsecond solves, graceful per-leg clamping
   instead of all-or-nothing tick rollback (see [Runtime IK](#runtime-ik-moveitpy) and
   [Phase 1](#phase-1--correctness--foundations)).
2. **Twist-based steering** — replace the weighted position-mixing of stride and rotation with
   per-foot displacement from a commanded body twist $(v_x, v_y, \omega_z)$
   (see [Steering](#steering--stride-composition)).
3. **Time-parameterized gait phase** — advance phase by measured $\Delta t$, decoupling robot
   speed from tick rate and from the lookahead-window side effect that currently doubles the
   phase rate (finding [F2](#findings-index)).
4. **Zero-velocity touchdown swing profile** — the current sine lifts the foot into the ground at
   maximum descent rate; switch to a cycloid or quintic Bézier
   (see [Gait generation](#gait-generation)).
5. **Contact sensing from servo feedback** — the A1-16 servos report load; contact detection
   unlocks terrain adaptation and real balance, which no amount of IK tuning can
   (see [Industry practice primer](#industry-practice-primer-for-hexapod-locomotion)).

## Architecture as it stands

```text
joystick / MCP → MovementCommand (/robot/movement_command)
        │
        ▼
HexapodBrain.loop()  @ 8 Hz  (brain_node.py)
        │  IMU tilt → apply_imu_balance / imu_balance_stride_scale
        ▼
WalkController.next_step_targets()  (walk_controller.py)
        │  ParametricGaitGenerator: phase → per-leg (x, z) offsets
        │  steering: direction transform + z-rotation, weighted mix
        ▼
AnalyticLocomotionKinematics.solve()  (locomotion_kinematics.py)
        │  per-leg closed-form IK → workspace/joint-limit clamping
        │  model→URDF conversion → MoveIt planning-scene collision check
        ▼
JointTrajectoryBuilder → /joint_trajectory_controller  (2-point window)
        │
        ▼
ros2_control → A1-16 servos / Gazebo        shadow FK: apply_joint_targets()
```

| Layer                   | Where                                                       | Maturity                                                     |
| ----------------------- | ----------------------------------------------------------- | ------------------------------------------------------------ |
| Analytic FK/IK model    | `drqp_kinematics/models.py`, `geometry/`                    | {bdg-success}`runtime` closed form with limit-aware clamping |
| Runtime IK + validation | `drqp_brain/locomotion_kinematics.py`, `drqp_moveit/config` | {bdg-success}`split` analytic IK, MoveIt collision oracle    |
| Gait generation         | `drqp_brain/parametric_gait_generator.py`                   | {bdg-success}`solid` standard phase-offset scheme            |
| Steering / composition  | `drqp_brain/walk_controller.py`                             | {bdg-warning}`heuristic` position mixing, unit mismatch      |
| Balance                 | `drqp_brain/balance_controller.py`                          | {bdg-warning}`early` P-only attitude, no contact             |
| Stride limits           | `stride_limits.py`, `generate_stride_limits.py`             | {bdg-success}`ahead of peers` translation-only today         |
| Execution               | `joint_trajectory_builder.py`, `brain_node.py`              | {bdg-warning}`low rate` 8 Hz, magic offsets                  |

The kinematics theory behind the analytic layer is covered in the
[kinematics model notebooks](kinematics-model.md).

## Analytic kinematics layer

_Package: `drqp_kinematics`_

`LegModel.inverse_kinematics()` is the standard closed form for a yaw–pitch–pitch leg: coxa
angle from `atan2(y, x)`, then a two-link planar solve by law of cosines in the (radial,
vertical) plane. `safe_arccos` clamps out-of-domain inputs to the boundary, which naturally
produces a "closest reach" pose for unreachable targets and reports solvability as a flag. This
is exactly how production hexapod firmwares (Phoenix-lineage, OpenSHC) solve legs, and it is
correct here.

What holds this layer back from being the runtime solver:

- **The solvability flag is dropped.** {bdg-secondary}`F8` `WalkController.__move_feet` ignores
  `move_to()`'s return, and nothing upstream consumes it. The layer already knows when a target
  is out of reach — the knowledge just never travels.
- **No joint-limit awareness.** {bdg-warning}`F9` The closed form returns the one elbow
  configuration it computes, with no check against the URDF limits (−90…90° coxa, −98…90° femur,
  −80…110° tibia — currently duplicated as a constant in `generate_stride_limits.py`). Limit
  checking is the one thing MoveIt currently adds that the analytic path must absorb before
  promotion.
- **Degrees at the API surface.** {bdg-secondary}`F10` The model speaks degrees, ROS speaks
  radians, and the servo layer adds its own offset constants. Every boundary is a conversion
  site.
- **Per-call overhead.** {bdg-secondary}`F11` `forward_kinematics()` allocates a chain of
  `AffineTransform`/`Point3D` objects and builds matplotlib-flavored label strings
  (`rf'$\alpha$=…'`) on every solve — fine for notebooks, wasteful in a control loop. A
  vectorized (6×3 array) FK/IK over all legs at once is the natural refactor.

:::{tip}
**Industry practice.** For 3-DOF point-foot legs, closed-form IK with explicit limit checking
and workspace clamping is the norm, even on research platforms; numeric IK is reserved for
≥6-DOF arms or when orientation constraints make the closed form intractable. The usual pattern:
solve analytically, clamp the target to the reachable annulus (a sphere-shell intersection per
leg) _before_ solving, and treat "clamped" as a soft signal for the gait layer rather than a
failure.
:::

## Runtime IK (MoveItPy)

_Files: `drqp_brain/locomotion_kinematics.py`, `drqp_moveit/config/`_

The runtime path builds an in-process MoveItPy from the node's own parameters, seeds a
`RobotState` from measured `/joint_states` (a hard-won correctness lesson — the cold-seed
freeze), solves each leg with `set_from_ik` against the KDL position-only plugin, retries once
from the home seed, then validates the complete 18-joint state for bounds and self-collision.
The engineering around the solver is careful. The solver choice is the problem.

- {bdg-danger}`F1` **Timeout budget is ~200× the loop budget.** `MOVEIT_IK_TIMEOUT_SEC = 2.0` is
  passed per `set_from_ik` call. A tick solves up to 6 legs × 2 window points, each with a
  possible retry — a worst-case tick can block the mutually-exclusive loop callback for tens of
  seconds against a 125 ms budget. The `kinematics.yaml` value (0.05 s) is what was clearly
  intended; the Python constant overrides it. Even in the current architecture this constant
  should drop to ≤ 0.02 s.
- {bdg-warning}`F3` **Numeric IK is nondeterministic in configuration space.** KDL's iterative
  solver converges to whichever branch the seed basin contains; a mid-swing seed near a
  workspace edge can flip femur/tibia branches between ticks, which the position-only check
  can't see. The retry-from-home fallback treats the symptom. An analytic solver has no basins.
- {bdg-warning}`F4` **Binary failure semantics freeze the robot.** One leg's IK failure rolls
  back the whole tick (`_restore_motion_state`), so the robot stalls in place and re-attempts
  the same phase forever until the command changes. Best practice is graceful degradation: clamp
  the offending foot to its nearest reachable point, keep the other five legs tracking, and
  surface a "stride saturated" signal that the steering layer can respond to (which
  `imu_balance_stride_scale` already does for one specific cause).
- {bdg-warning}`F5` **Validation is on the wrong side of the loop.** Bounds + self-collision
  checking of every candidate state is a planning-time activity. At runtime, geometry that was
  proven safe offline (which is exactly what `stride_limits.yaml` certifies) doesn't need
  re-proving 8 times a second; what runtime needs is fast clamping. Keep the MoveIt validation
  as (a) the offline calibration oracle, (b) a CI property test over the gait envelope, and
  optionally (c) a low-rate async watchdog.

:::{tip}
**Implemented architecture: analytic first, MoveIt as oracle.** `drqp_kinematics` is the
default runtime solver and returns per-leg joint targets with clamping metadata. The selectable
`moveit` backend remains available for A/B comparison, stride-limit generation, and launch
tests. A slow oracle test asserts both solvers agree across the certified gait envelope. The
runtime still assembles the analytic targets into a MoveIt `RobotState` for self-collision
checking; removing that final hot-path safety net remains gated on certification of combined
translation, rotation, body-pose, and balance commands. An alternative would be an analytic
`kinematics::KinematicsBase` plugin (IKFast-style) — but for six identical 3-DOF chains, plain
Python/numpy is simpler and faster.
:::

## Gait generation

_File: `drqp_brain/parametric_gait_generator.py`_

The generator is the classic phase-offset scheme: each gait is a table of per-leg swing start
phases plus a swing duration (duty factor = 1 − swing duration). The tables are correct —
tripod ½, ripple ⅓ swing with the proper left/right interleave, wave ⅙ — and the parametric
form (unit stride scaled downstream) is a good design that made the offline calibration
possible. See also the [gait generation notebook](../notebooks/3_generating_gaits.md).

- {bdg-warning}`F6` **Touchdown at maximum descent velocity.** Swing height is
  $z = \sin(\pi t)$, whose derivative at $t = 1$ is $-\pi$ — the foot hits the ground at its
  _peak_ vertical speed, and swing x is linear so the horizontal velocity is discontinuous at
  both liftoff and touchdown (the foot must instantly reverse from forward swing speed to stance
  speed). In simulation this shows up as impact bounce and foot slip; on hardware it is servo
  stress and position error precisely at the moment the leg takes load. Standard fix: a profile
  with zero velocity (ideally zero acceleration) at both ends — cycloid
  ($x \propto t - \sin(2\pi t)/2\pi$, $z \propto (1 - \cos(2\pi t))/2$) or a quartic/quintic
  Bézier as used by OpenSHC and Phoenix-lineage firmwares. This is a ~15-line change confined to
  `get_offsets_at_phase_for_leg`.
- {bdg-warning}`F12` **Tripod has zero double-support margin.** With swing duration exactly ½,
  there are instants where support switches between tripods with no overlap; any timing jitter
  or early liftoff momentarily leaves the robot on < 3 legs. Real deployments use duty
  ≈ 0.52–0.55 for tripod. Making duty a parameter (per gait, eventually speed-dependent) also
  opens the door to smooth gait transitions.
- {bdg-secondary}`F13` **Phase wraparound epsilon.** `leg_phase %= 1.000001` works but distorts
  the cycle length by 1e-6 and reads as a mystery; `math.fmod(leg_phase, 1.0)` with an explicit
  half-open convention says what it means.
- {bdg-secondary}`F14` **Gait switches are discontinuous.** Changing `current_gait`
  re-interprets the same global phase under a different offset table, so feet can teleport
  between swing and stance. A transition scheme (finish current cycle, or remap each leg to the
  nearest equivalent phase in the new gait) belongs in the roadmap once stop/start sequencing
  exists.

## Steering & stride composition

_File: `drqp_brain/walk_controller.py`_

Translation is handled by rotating a nominal x-stride into the commanded direction — fine.
Rotation reuses the gait phase as a turn parameter and rotates each neutral foot position about
body-z — also fine in isolation. The problems are in how the two combine and in the smoothing:

- {bdg-warning}`F7` **Weighted position mixing distorts both commands.**
  `foot_target = stride·w_s + rotation·w_r` with weights normalized to sum to 1 means commanding
  any rotation proportionally _reduces_ translation (and vice versa), and the two inputs live in
  different units (stride ratio vs. rotation fraction of 45°/step). The result walks, but the
  mapping from joystick to body motion is nonlinear and speed-dependent, and stride-limit
  clamping (computed for pure translation) no longer bounds the combined motion — the exact gap
  the issue #397 test is probing.
- {bdg-warning}`F15` **Frame-rate-dependent smoothing.** `interpolate(target, 0.3)` per tick is
  an exponential filter whose time constant is welded to fps; change the loop rate and the
  robot's responsiveness changes. Express it as a time constant
  ($\alpha = 1 - e^{-\Delta t/\tau}$) or, better, a slew-rate limit on the twist (max
  linear/angular acceleration), which is the standard velocity-command shaping.
- {bdg-warning}`F16` **L1 norms make steering anisotropic.** Stride magnitude is computed as
  $|x|+|y|$ in three places (walk controller ×2, stride-limit clamp). A 45° joystick command
  yields ~41% more commanded stride than a straight-ahead one before clipping. Use the Euclidean
  norm.
- {bdg-warning}`F17` **Stopping lurches.** When motion falls below `no_motion_eps`, targets snap
  to the stored neutral tips in a single tick — all six feet slide home simultaneously,
  mid-cycle. Starting resets phase to 0 regardless of leg poses. A small gait sequencer state
  machine (starting → walking → stopping, where stopping finishes the half-cycle and steps each
  swing leg to neutral) is the established pattern.

:::{tip}
**Industry practice — twist-based steering.** Treat the command as a planar body twist
$\xi = (v_x, v_y, \omega_z)$. For each leg with neutral foot position $r_i$, the ground velocity
of the foot relative to the body during stance is $u_i = -(v + \omega \times r_i)$; the stride
vector is $u_i$ integrated over stance time, and swing retraces it through the swing profile.
This single formula replaces the direction transform, the rotation transform, and the mixing
weights; translation and rotation compose exactly; each leg gets a _different, correct_ stride
(outer legs sweep farther in a turn, which the current shared-phase rotation approximates); and
reachability can be enforced per leg by scaling $\xi$ — one scalar — until every stride fits its
workspace. It also gives you dead-reckoned odometry for free (integrate the same twist).
OpenSHC, Free Gait descendants, and every modern quadruped controller steer this way.
:::

## IMU balance

_Files: `drqp_brain/balance_controller.py`, `drqp_brain/brain_node.py`_

The current design: recover body orientation from the Gazebo IMU through the fixed mount
rotation, capture a target tilt when balance mode engages, apply a clipped proportional
roll/pitch correction as an extra body rotation, and scale the stride down when the correction
saturates. The mount-frame handling is correct and well-documented, the target-capture semantics
are sensible, and the stride-backoff coupling is a genuinely good idea (graceful degradation
instead of repeated IK failure). See also the
[IMU body balancing notebook](../notebooks/4_imu_body_balancing.md).

- {bdg-warning}`F18` **P-only, unfiltered, at 8 Hz.** Gain 2.0 with a 0.15 rad clamp on a
  proportional loop with no derivative term, no deadband, and no smoothing of the measurement
  will oscillate on hardware where the IMU is noisy and the plant has servo lag — Gazebo's clean
  quaternions hide this. Minimum viable: low-pass the tilt estimate, add a
  derivative-on-measurement term, and rate-limit the correction. The correction should also be
  slewed when balance mode toggles, so engagement doesn't step the body pose.
- {bdg-warning}`F19` **Attitude-only balance can't see the feet.** Rotating the body frame
  re-levels the torso but assumes all six feet stay planted; on uneven ground the actual need is
  per-leg height adaptation, which requires contact information. The current controller is a
  posture controller, not a balance controller — worth naming as such in code/docs so its scope
  stays honest.
- {bdg-secondary}`F20` **Comment/behavior mismatch in the backoff ramp.** With `floor = 0.3`,
  `1 − (saturation − 1)` reaches the floor at saturation 1.7×, not the "double the clamp" the
  comment promises. Trivial, but this is exactly the kind of constant someone will tune later by
  reading the comment.
- {bdg-secondary}`F21` **Real-IMU orientation quality is unowned.** `imu_node` hands through
  backend orientation when present; when a backend provides only gyro+accel there is no fusion
  fallback. Adopting a complementary or Madgwick filter (or `imu_filter_madgwick`) as an
  explicit stage makes hardware behavior predictable.

:::{tip}
**Industry practice — stability ladder.** Hexapods climb a well-trodden ladder: (1) posture
control (you are here); (2) **static stability margin** — track the support polygon of stance
feet and keep the projected CoM inside it with margin, shifting the body or modulating gait when
it shrinks (this is what actually matters for wave gait on slopes); (3) contact-reactive
stepping — extend/shorten swing on early/late touchdown; (4) ZMP/dynamic criteria — only needed
at speeds hexapods rarely reach. Skipping (2) and (3) to tune (1) harder is the common dead end.
:::

## Stride-limit calibration

_Files: `drqp_brain/stride_limits.py`, `drqp_brain/generate_stride_limits.py`_

Precomputing a per-gait polar table of maximum safe step lengths by binary-searching full IK
sweeps with a joint margin — this is a reachability map, and having it at all puts this codebase
ahead of most hexapod projects. The YAML versioning, validation, and interpolation code are
clean. Gaps, all known or knowable:

- {bdg-warning}`F22` **The table is translation-only.** Calibration sweeps
  `rotation_direction=0.0`, so combined stride+rotation (and any body-pose offset, and the IMU
  balance correction) can exceed certified limits — the runtime then relies on IK failure +
  rollback as the safety net. Options, in increasing order of elegance: extend the grid with a
  rotation axis; certify a conservative combined envelope; or — with twist steering — replace
  the table entirely with an exact per-leg analytic workspace check at runtime, keeping the
  offline sweep as CI regression.
- {bdg-secondary}`F23` **Joint limits duplicated by hand.** `JOINT_LIMITS_DEGREES` mirrors the
  URDF with a comment begging them to stay in sync. Parse them from the already-loaded
  `robot_description` instead.
- {bdg-secondary}`F24` **Regeneration is manual.** A CI job that regenerates the table and diffs
  it against the committed YAML would catch geometry/config drift the moment it happens.

## Control loop & execution

_Files: `drqp_brain/brain_node.py`, `drqp_brain/joint_trajectory_builder.py`_

- {bdg-warning}`F2` **The lookahead window double-advances the gait.** Each tick calls
  `next_step_targets` once for "now" and once more to fill the 2-point trajectory window — both
  calls mutate `current_phase`. Every successful tick therefore advances the gait by _two_ phase
  steps while ticks arrive at 1× fps, and the second trajectory point is usually preempted by
  the next publish. Consequences: actual cycle time is half what `phase_steps_per_cycle`
  implies, and walking speed silently changes if `walking_trajectory_points` changes. The fix
  falls out of time-parameterization: compute targets as a pure function `targets(phase)`,
  evaluate it at $t+1/\text{fps}$ and $t+2/\text{fps}$ for the window, and commit phase once per
  real tick from measured $\Delta t$.
- {bdg-warning}`F25` **8 Hz is an order of magnitude below practice.** Hobby hexapod firmwares
  run 30–50 Hz command loops; research platforms 100–500 Hz. The joint trajectory controller's
  spline interpolation papers over it, but steering latency (~250 ms with the smoothing filter)
  and balance bandwidth are capped by it. With analytic IK, 50 Hz is comfortably reachable in
  Python (18 closed-form solves ≈ microseconds vectorized); the architecture then cleanly splits
  into gait/IK at 50 Hz and higher-level decisions at 10 Hz.
- {bdg-success}`F26` **Analytic-model and URDF zero conventions have one boundary.**
  `drqp_kinematics` owns the fixed femur- and tibia-bracket conversion between the simplified
  straight-link analytic model and the physical URDF. Model-generated poses cross that boundary
  explicitly before trajectory construction; MoveIt results are already in URDF convention and
  never receive the conversion. These assembly offsets are not per-servo trim calibration.
- {bdg-secondary}`F27` **Dead constructor parameter.** `phase_steps_per_cycle=self.fps / 2.5` at
  construction is overwritten every tick from the per-gait list; it only misleads.
- {bdg-secondary}`F28` **Dedupe key as change detection.** `_foot_targets_window_key` (rounded
  floats over the whole window plus body transform) is a symptom of the loop not knowing whether
  anything changed; with a pure `targets(phase, twist, pose)` function, "did the inputs change"
  is a direct comparison of a handful of state variables.

## Industry practice primer for hexapod locomotion

A condensed map of the techniques the roadmap draws from, ordered roughly by the layer they
touch.

### Leg IK & workspace

Closed-form yaw–pitch–pitch IK with limit-aware branch selection; precomputed per-leg workspace
(annulus between $|l_{femur} - l_{tibia}|$ and $l_{femur} + l_{tibia}$ in the coxa plane,
intersected with joint limits); target clamping to the workspace boundary as the universal
graceful-degradation primitive. Numeric IK and full planners stay offline.

### Steering & gait

**Twist-based stride computation** is the backbone. On top of it: duty factor and stride
frequency as functions of commanded speed (constant swing _time_, variable stance time); gait
selection by speed/terrain (wave → ripple → tripod as speed rises); phase-remapped gait
transitions; a start/stop sequencer. **CPG-based generators** (coupled oscillators) are the main
alternative school — they buy smooth transitions and biological plausibility but make precise
foot placement and workspace certification harder; for a robot with this calibration
infrastructure, the parametric/twist approach is the better fit. The **Raibert heuristic**
(shift touchdown point proportional to velocity error) is worth adopting even for statically
stable gaits: it turns foot placement into a stabilizing feedback and costs one line per swing
target.

### Swing trajectories

Zero velocity and acceleration at liftoff/touchdown (cycloid or quintic Bézier); ground-speed
matching at touchdown so the foot lands already moving at stance velocity relative to the body
(eliminates scuffing); obstacle-clearance shaping (raise apex, not endpoints). OpenSHC's
node-based Bézier swing is the reference open implementation.

### Stability & terrain

Static stability margin over the support polygon; body CoM shift during wave/ripple gait (the
classic hexapod refinement — shift the body toward the supporting side before lifting a leg);
contact detection from servo load/current or position error (the A1-16's feedback is
sufficient — threshold the PID error during late swing); contact-reactive swing termination
("step down until touch"), which is the single biggest robustness win on uneven ground; per-leg
virtual compliance (admittance on foot z) once contact estimation exists.

### State estimation

Complementary/Madgwick filter for attitude on real IMUs; leg odometry from stance-foot
kinematics fused with IMU (velocity estimate → enables Raibert correction and closes the teleop
loop); slip detection from disagreement between leg odometry and expected twist.

### Architecture & verification

Separate rates: command shaping (10–20 Hz) → gait/IK (50–100 Hz) → servo bus (whatever the
A1-16 chain sustains). Pure functions for gait math (the codebase is close —
`targets(phase, twist, pose)` with state committed explicitly). Property tests: FK∘IK
round-trip identity over the workspace; analytic-vs-MoveIt agreement; golden gait traces (foot
positions over one cycle per gait) as regression fixtures; the existing launch-test culture
already covers the integration layer well.

## Development roadmap

### Phase 1 — correctness & foundations

Each item is independently shippable.

1. **Clamp the IK timeout** to ≤ 0.02 s (`locomotion_kinematics.py:34`) — one line, removes the
   multi-second stall mode immediately. {bdg-danger}`F1`
2. **Fix the phase double-advance**: make target generation a pure function of phase, evaluate
   the window without mutating state, advance phase once per tick by measured $\Delta t$.
   {bdg-warning}`F2` {bdg-warning}`F15`
3. **Promote analytic IK** (complete): limit-aware `LegModel` solves the hot loop with per-leg
   clamping; MoveIt remains the selectable fallback, collision validator, calibration tool, and
   agreement oracle. {bdg-success}`F3` {bdg-success}`F4` {bdg-warning}`F5`
4. **Cycloid swing profile** in `get_offsets_at_phase_for_leg` (complete).
   {bdg-success}`F6`
5. **Twist-based steering** in `WalkController`: per-leg stride from $(v, \omega)$, Euclidean
   norms, twist-level stride-limit scaling. {bdg-warning}`F7` {bdg-warning}`F16`
6. **Keep servo zero offsets single-source** in `drqp_kinematics`; trajectory construction
   consumes URDF/controller radians only. {bdg-success}`F26`

### Phase 2 — robustness & feel

1. **Raise the loop to 30–50 Hz** once analytic IK lands; split command shaping from gait
   execution. {bdg-warning}`F25`
2. **Gait sequencer**: starting/stopping states, step-to-neutral on stop, phase-remapped gait
   switching, tripod duty 0.52–0.55. {bdg-warning}`F17` {bdg-warning}`F12` {bdg-secondary}`F14`
3. **Contact detection** from A1-16 load/position-error; expose per-leg contact state as a
   topic; contact-reactive swing termination.
4. **Balance v2**: filtered tilt, PD control with slewed engagement, then
   static-stability-margin monitoring with body CoM shift for wave/ripple. {bdg-warning}`F18`
   {bdg-warning}`F19`
5. **Stride limits v2**: derive from analytic workspace at runtime (per-leg, includes rotation
   and body pose exactly); keep the MoveIt sweep as a CI regression that regenerates and diffs
   the YAML. {bdg-warning}`F22` {bdg-secondary}`F24`
6. **Stance/posture parameters**: named body height / foot radius stance config replacing the
   hard-coded `forward_kinematics(0, −35, 130)` pose and the `body_translation / 8.0` scaling.

### Phase 3 — capability

1. **Leg odometry + velocity estimation**; publish `odom`; Raibert-style touchdown correction on
   the velocity error.
2. **Terrain adaptation**: per-leg height offsets from contact history; virtual-plane fitting of
   stance feet; body attitude tracking the terrain plane rather than gravity on slopes.
3. **Per-leg admittance** (soft z-compliance) using servo effort — the step from
   "position-controlled toy" to "compliant walker".
4. **Free-gait / footstep planning** for obstacle-aware stepping, only after contact + odometry
   exist; MoveIt's planning stack becomes genuinely useful here (whole-body collision-checked
   poses for e.g. leg-waving behaviors, body-pose manipulation while standing).
5. **Learning-based extensions** (RL policies for rough terrain, MPC for dynamic transitions) —
   viable in the existing Gazebo pipeline, but only worth it once the classical stack above is
   the baseline it competes against.

## Findings index

| ID  | Severity               | Finding                                                                                                                                    | Where                                                        |
| --- | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------ |
| F1  | {bdg-danger}`critical` | 2.0 s per-call IK timeout vs 125 ms loop budget                                                                                            | `locomotion_kinematics.py:34,143`                            |
| F2  | {bdg-warning}`high`    | Lookahead window mutates phase → gait runs at 2× configured rate; speed coupled to `walking_trajectory_points`                             | `brain_node.py:417–437`                                      |
| F3  | {bdg-success}`fixed`   | Analytic IK is the default; numeric MoveIt IK remains a selectable fallback and oracle                                                     | `locomotion_kinematics.py`, `brain_node.py`                  |
| F4  | {bdg-success}`fixed`   | Per-leg workspace/limit clamping publishes degraded motion and reports persistent saturation                                               | `locomotion_kinematics.py`, `brain_node.py`                  |
| F5  | {bdg-warning}`partial` | Analytic bounds enforcement replaced runtime MoveIt bounds checks; self-collision validation remains until combined-envelope certification | `locomotion_kinematics.py`                                   |
| F6  | {bdg-success}`fixed`   | Cycloid swing profile enforces zero x/z velocity at liftoff and touchdown ([PR #441](https://github.com/Dr-QP/Dr.QP/pull/441))          | `parametric_gait_generator.py`                               |
| F7  | {bdg-warning}`high`    | Stride/rotation combined by normalized position averaging — unit mismatch, mutual attenuation, uncertified combined envelope               | `walk_controller.py:175–181`                                 |
| F8  | {bdg-secondary}`low`   | Analytic IK solvability flag ignored by callers                                                                                            | `walk_controller.py:224–226`, `models.py:255`                |
| F9  | {bdg-success}`fixed`   | Analytic IK parses URDF limits and clamps each leg in model convention                                                                     | `models.py`, `urdf_limits.py`                                |
| F10 | {bdg-secondary}`low`   | Degrees/radians/servo-offset conversions at every layer boundary                                                                           | `models.py`, `joint_trajectory_builder.py`, `brain_node.py`  |
| F11 | {bdg-secondary}`low`   | FK allocates transform chains + label strings per call in the control path                                                                 | `models.py:219–252`                                          |
| F12 | {bdg-warning}`medium`  | Tripod duty exactly 0.5 — zero double-support margin                                                                                       | `parametric_gait_generator.py:97–107`                        |
| F13 | {bdg-success}`fixed`   | Half-open `% 1.0` phase wrapping preserves negative-phase behavior without a cycle-length epsilon ([PR #441](https://github.com/Dr-QP/Dr.QP/pull/441)) | `parametric_gait_generator.py`                               |
| F14 | {bdg-secondary}`low`   | Gait switching discontinuous (no phase remap/transition)                                                                                   | `walk_controller.py:53–55`                                   |
| F15 | {bdg-warning}`medium`  | Per-tick 0.3 exponential smoothing — time constant welded to fps                                                                           | `walk_controller.py:117–121`                                 |
| F16 | {bdg-warning}`medium`  | L1 norms for stride magnitude → diagonal anisotropy                                                                                        | `walk_controller.py:108–129`, `stride_limits.py:123`         |
| F17 | {bdg-warning}`medium`  | Stop snaps all feet to neutral in one tick; start resets phase blindly                                                                     | `walk_controller.py:138–143, 183`                            |
| F18 | {bdg-warning}`medium`  | Balance is P-only, unfiltered, un-slewed, at 8 Hz — oscillation risk on hardware                                                           | `balance_controller.py:97–121`, `brain_node.py:97`           |
| F19 | {bdg-warning}`medium`  | Attitude-only correction; no support-polygon/contact awareness                                                                             | `balance_controller.py`                                      |
| F20 | {bdg-secondary}`low`   | Backoff ramp comment ("double the clamp") disagrees with math (1.7×)                                                                       | `balance_controller.py:92–94`                                |
| F21 | {bdg-secondary}`low`   | No fusion fallback when an IMU backend lacks on-chip orientation                                                                           | `imu_node.py`                                                |
| F22 | {bdg-warning}`medium`  | Stride limits certify translation only — rotation/body-pose/balance offsets uncovered                                                      | `generate_stride_limits.py:184–189`                          |
| F23 | {bdg-secondary}`low`   | Joint limits hand-duplicated from URDF                                                                                                     | `generate_stride_limits.py:48–52`                            |
| F24 | {bdg-secondary}`low`   | Stride-limit YAML regeneration not CI-enforced                                                                                             | —                                                            |
| F25 | {bdg-warning}`medium`  | 8 Hz control loop caps steering latency and balance bandwidth                                                                              | `brain_node.py:93`                                           |
| F26 | {bdg-warning}`medium`  | Servo zero offsets applied/inverted in trajectory layer instead of URDF/ros2_control                                                       | `joint_trajectory_builder.py:35–36`, `brain_node.py:493–501` |
| F27 | {bdg-secondary}`low`   | Dead `phase_steps_per_cycle` constructor argument (fps/2.5), overwritten every tick                                                        | `brain_node.py:227`                                          |
| F28 | {bdg-secondary}`low`   | Rounded-float dedupe key compensating for stateful target generation                                                                       | `brain_node.py:456–476`                                      |

## Suggested reading

- **OpenSHC / Syropod High-level Controller** (CSIRO) — the closest open-source reference for
  everything in the practice primer: twist steering, Bézier swings, gait sequencing, admittance,
  pose control. Its paper ("OpenSHC: A Versatile Multilegged Robot Controller", IEEE Access 2020) doubles as a design document.
- **Siciliano & Khatib (eds.), Springer Handbook of Robotics** — chapter on legged robots for
  stability margins and gait theory.
- **Raibert, Legged Robots That Balance** — the foot-placement heuristic and the mindset of
  velocity-stabilizing stepping.
- **Campos et al., "Hexapod Locomotion: A Nonlinear Dynamical Systems Approach"** and Ijspeert's
  CPG survey (Neural Networks, 2008) — if you explore the oscillator school.
- **Focchi et al., "Heuristic Planning for Rough Terrain Locomotion…"** and the ANYmal/HyQ line
  of papers — static stability margin + contact-reactive stepping done rigorously.
- **MIT Cheetah 3 / Mini Cheetah convex MPC papers** (Di Carlo et al., IROS 2018) — the dynamic
  end of the ladder, relevant only for Phase 3+ ambitions.
