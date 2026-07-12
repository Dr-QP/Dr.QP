# Locomotion migration specs

Implementation-ready specifications for migrating the Dr.QP locomotion stack in the direction
recommended by the [IK & locomotion analysis](../../../source/Dev/ik-locomotion-analysis.md):

> Make the analytic solver the primary runtime IK with explicit joint-limit and workspace
> clamping, keep MoveIt as an offline/CI validation oracle, reformulate steering as a body
> twist, and reinvest the freed CPU budget in loop rate, swing-profile quality, and
> contact-aware balance.

Finding IDs (`F1`…`F28`) referenced throughout are defined in the
[findings index](../../../source/Dev/ik-locomotion-analysis.md#findings-index) of that page.

> **This program is the foundation for the [autonomy roadmap](../roadmap/README.md).** It
> removes the MoveItPy hot path, makes gait phase time-based, and reformulates steering as a
> metric body twist — the substrate several roadmap phases assume. See
> [Program relationships](../README.md#program-relationships) for the hand-off points — notably
> that spec 06 supersedes the empirical velocity model RM-02 would otherwise fit, and spec 08's
> `control_rate_hz` replaces the hardcoded 8 Hz the roadmap phases were written against.

## Implementation order

Specs are numbered in implementation order. Specs 01, 02, 03, 04, and 07 have landed and their
implementation files have been removed. Remaining specs assume those foundations unless the
dependency table says otherwise.

| Spec                                                   | Title                                              | Fixes                  | Depends on |
| ------------------------------------------------------ | -------------------------------------------------- | ---------------------- | ---------- |
| [05](05-cycloid-swing-profile.md)                      | Cycloid swing profile                              | F6, F13                | 02 (soft)  |
| [06](06-twist-based-steering.md)                       | Twist-based steering                               | F7, F16, F22 (partial) | 02, 03, 04 |
| [08](08-raise-control-loop-rate.md)                    | Raise control loop rate                            | F25                    | 02, 04     |

Dependency sketch:

```text
01 ──────────────────────────────► landed

02 ─┬─► 04 ─┬─► 06
03 ─┘       ├─► 07 (landed)
05 (after 02, parallel with 03/04)
            └─► 08 (also needs 02)
```

## Conventions for implementing agents

- **Workflow**: follow the TDD cycle (`TDD Red` → `TDD Green` → `TDD Refactor` agents, or the
  equivalent manual discipline). Every spec lists the tests to write first.
- **Build/test**: always through `scripts/with-ros-env.sh`, e.g.
  `scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to drqp_brain` and
  `scripts/with-ros-env.sh colcon test --packages-select drqp_brain drqp_kinematics`.
- **Launch-test validation**: changes touching MoveIt, IK seeding, or trajectory publication
  must be validated against the real Gazebo launch tests
  (`test_bringup_launch.py`, `test_brain_moveit_ik.py`) — unit-test fakes do not catch
  moveit_py C++ crashes.
- **Behavior preservation**: unless a spec explicitly says otherwise, observable walking
  behavior (gait cycle time, stride reach, topics, message types) must not change. Where a spec
  intentionally changes behavior, it says so under _Behavior changes_ and lists the tunables to
  re-tune.
- **Docs**: when a spec lands, update the maturity/finding status in
  `docs/source/Dev/ik-locomotion-analysis.md` (mark the finding fixed with the PR number) and
  regenerate `stride_limits.yaml` if leg geometry, IK, or gait math changed.
- **One spec per PR.** Keep diffs reviewable; specs are sized to be independently shippable.
