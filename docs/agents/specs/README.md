# Dr.QP agent specs

Implementation-ready, machine-oriented specifications for evolving Dr.QP. Each spec is sized to
be independently shippable (one spec ≈ one PR) and lists the tests to write first. Human-narrative
counterparts, where they exist, live under `docs/source/Dev/`.

The specs are grouped into **programs** — independent bodies of work but not unrelated — see
[Program relationships](#program-relationships).

| Program                                                                 | What it does                                                               | Source of rationale                                                       |
| ----------------------------------------------------------------------- | -------------------------------------------------------------------------- | ------------------------------------------------------------------------- |
| [`locomotion/`](locomotion/README.md)                                   | Foundation refactor: analytic-IK hot path, twist steering, time-based gait | [`ik-locomotion-analysis.md`](../../source/Dev/ik-locomotion-analysis.md) |
| [`roadmap/`](roadmap/README.md)                                         | Autonomy roadmap: joystick hexapod → autonomous AI home pet                | [`roadmap/index.md`](../../source/Dev/roadmap/index.md)                   |
| [`formatting-linting/`](formatting-linting/README.md)                   | Unify formatter ownership, local/editor workflows, CI, and ROS lint gates  | (repository tooling audit, self-contained)                                |
| [`containerized-ai-responders/`](containerized-ai-responders/README.md) | Run selectable Claude/Codex responders in the ROS CI container             | (repository tooling audit, self-contained)                                |
| [`chatgpt-codex-auth-ci/`](chatgpt-codex-auth-ci/README.md)             | Decision record and private-only design for ChatGPT-managed Codex CI auth  | OpenAI CI/CD authentication guidance and workflow audit                   |

## Program relationships

The programs can be worked in parallel by different agents, but they touch overlapping code and
one program de-risks the others. Read this before starting cross-program work.

### Locomotion migration is the substrate the roadmap assumes

The autonomy roadmap (RM-\*) was written against **today's** walking stack (8 Hz MoveItPy loop,
position-mixed steering). The locomotion migration replaces exactly that stack. Where they meet:

- **Metric velocity (RM-02 ⇄ locomotion 06).** RM-02 proposes fitting an _empirical_
  `v_body = f(stride_length, phase_rate)` model from sim ground truth. Locomotion spec 06 makes
  velocity metric _by construction_ — the commanded twist `ξ = (v_x, v_y, ω_z)` **is** the body
  velocity via the SE(2) exponential foot-target map. **If spec 06 lands first, RM-02 collapses to
  a thin `Twist → ξ` adapter plus `twist_mux`/watchdog/arbitration** and the empirical fit becomes
  a validation cross-check rather than the mechanism. RM-02 says as much in its updated "Design"
  note; do not build the empirical mapper if spec 06 is already in.
- **Loop rate (RM-02 "8 Hz" ⇄ locomotion 08).** Several roadmap phases assume `fps = 8`.
  Locomotion spec 08 turns that into a `control_rate_hz` parameter (default 25). Anything reading a
  hardcoded 8 Hz should read the parameter once spec 08 lands.
- **Analytic hot path (RM-08/RM-09 ⇄ locomotion 04).** RM-08 touchdown adaptation and RM-09's
  runtime both assume the MoveItPy per-tick solve is gone. That removal is locomotion spec 04.
- **Stance/odometry (RM-03/RM-08 ⇄ locomotion 02/06).** RM-03 leg odometry and its `GaitPhase`
  message consume the phase/stance state that locomotion spec 02 makes explicit and side-effect
  free; RM-06's twist odometry option (spec 06) is the open-loop counterpart RM-03's EKF fuses.

**Recommended global order:** land the locomotion foundation (at least specs 01–04, ideally
through 06/08) before or alongside RM-02, so the roadmap builds on the analytic twist stack rather
than the stack it is about to replace. RM-01 (baseline/hardening) and RM-04 (camera) have no such
coupling and can proceed immediately.

### Stance detection has three sources: keep them one interface

`leg_in_stance` appears in locomotion spec 02 (gait-phase truth), RM-03's `GaitPhase` message, and
RM-08's `FootContacts` (sensed). These are a _source hierarchy_ for one concept, not three
independent features: gait-phase stance is the default, sensed contact overrides it when
available (RM-08's `stance_source` parameter). Consumers must never hardcode a source.

## Conventions (all programs)

- **Workflow:** TDD (`TDD Red` → `TDD Green` → `TDD Refactor`, or the manual equivalent). Every
  spec lists the tests to write first.
- **Build/test:** always through `scripts/with-ros-env.sh`, e.g.
  `scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to <pkg>` and
  `colcon test --packages-select <pkg>`. See [`AGENTS.md`](../../../AGENTS.md).
- **Simulation-first:** every capability lands in `drqp_gazebo` (worlds/bridge/`launch_pytest`
  tests) before hardware; MoveIt/IK/trajectory changes are validated on the real Gazebo launch
  stack — unit fakes do not catch moveit_py binding crashes.
- **Safety path stays local and untouched:** `/robot_event`, `/robot_state`, joint trajectories,
  and the kill switch keep their current semantics; heavy perception/LLM nodes never sit in the
  safety path.
- **One spec per PR.** Keep diffs reviewable.

Each program's own README carries its ordering table, per-program conventions, and (for the
locomotion program) the finding-ID map into the analysis document.
