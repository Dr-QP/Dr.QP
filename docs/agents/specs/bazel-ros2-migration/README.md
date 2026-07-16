# Bazel ROS 2 Jazzy migration

## Decision

Treat Bazel as a staged, optional build for a deliberately selected runtime
surface. Keep ament metadata and the colcon build, test, coverage, and release
paths authoritative until Spec 09 records a promotion decision.

Start from a repository-owned fork of `mvukov/rules_ros2`. The candidate
revision verified on 2026-07-15 is upstream `feature/jazzy` commit
`56ad1dfa6a636378e623dd5903c4ff4d7d2acd4b` (the head of open PR #558 at that
time). Spec 01 must test that exact revision, mirror it to an immutable commit
or signed tag in `Dr-QP/rules_ros2`, and record any later revision instead of
silently following the branch.

The fork is a supply-chain and maintenance boundary, not a commitment to
permanent divergence. Generic fixes belong upstream; workspace-specific or
not-yet-merged fixes must be listed in the fork's `UPSTREAM.md`.

## Raw findings

### Compatibility surface

The workspace has 17 ROS packages spanning generated messages, C++ and Python,
pluginlib, ros2_control, launch, Gazebo, MoveIt, xacro, package resources, and
patched `launch_pytest` behavior. Compilation alone does not prove the
following high-risk boundaries:

- C, C++, and Python ROSIDL generation for all five `drqp_interfaces` messages;
- cross-language type support and wire compatibility;
- pluginlib discovery of the `drqp_control` hardware plugin;
- ament-index, launch-file, xacro, mesh, YAML, and world lookup from runfiles;
- Python 3.12 imports and console entry points;
- the repository's vendored `launch_pytest` retry and process-exit behavior;
- Gazebo and MoveIt processes supplied outside the current rules fork.

PR #558 is still open and its discussion contains unresolved downstream
reports around simultaneous C/C++ interface generation and generated IDL
descriptions. Treat the candidate revision as untrusted until Specs 02 and 03
pass in this workspace.

### Issue register

- **P0 — rules coverage stops before ros2_control.** The candidate fork has no
  ros2_control/ros2_controllers repositories or BUILD adapters. Spec 03 owns
  that dependency layer; first-party control targets must not mix source-built
  rclcpp/pluginlib with binary hardware-interface libraries.
- **P0 — generated interfaces may conflict.** The upstream discussion reports
  simultaneous C/C++ generator action collisions. Spec 02 builds C, C++, and
  Python closures together and adds cross-language serialization/transport
  tests before any consumer migration.
- **P0 — plugin and runtime discovery can diverge from compilation.** The
  hardware plugin, ament index, xacro, config, meshes, worlds, and launch files
  need declared runfiles plus process-level tests in Specs 04–07.
- **P1 — launch retry semantics differ upstream.** This workspace vendors a
  launch-pytest fix at `7df946eec4ef5d24c427a16d72c19b368dd643d1`.
  Spec 05 makes its retry/event-loop and process-exit behavior a named gate.
- **P1 — Gazebo and MoveIt are host ABI boundaries.** The candidate rules do
  not build those stacks. Specs 06–07 isolate `/opt/ros/jazzy`, record versions,
  and reject workspace overlays; the result remains non-hermetic.
- **P1 — dual-build declarations can drift.** Every spec shares test sources
  and preserves ament metadata while adding explicit Bazel source/data labels.
- **P2 — developer and CI cost is unknown.** Specs 08–09 measure clean/no-op
  behavior, cache, disk, runner time, setup, diagnostics, and rollback before
  any required-check decision.

## Implementation order

Each spec is one main-targeted PR and is intended for one implementation agent.
Do not combine adjacent specs to obtain `bazel build //...`.

| Order | Spec                                                                            | Size | Outcome                                                                         |
| ----- | ------------------------------------------------------------------------------- | ---- | ------------------------------------------------------------------------------- |
| 1     | [01 — Fork and Bazel foundation](01-fork-and-bazel-foundation.md)               | M    | Immutable rules fork, Bazel 8.7.0 entry point, lockfiles, and isolation checks. |
| 2     | [02 — Generated interfaces](02-generated-interfaces.md)                         | M    | All custom messages build for C/C++/Python and pass cross-language tests.       |
| 3     | [03 — ros2_control rules support](03-ros2-control-rules-support.md)             | M/L  | Source-built ros2_control/controller targets and rules-level plugin proof.      |
| 4     | [04 — C++ libraries and control plugin](04-cpp-libraries-and-control-plugin.md) | M    | Serial/driver/control libraries and the hardware plugin build and load.         |
| 5     | [05 — Python, joy, and launch support](05-python-and-launch-support.md)         | M/L  | Python packages, joy, entry points, and patched launch support run under Bazel. |
| 6     | [06 — Gazebo runtime parity](06-gazebo-runtime-parity.md)                       | M/L  | Existing Gazebo smoke/full tests run from Bazel-owned first-party artifacts.    |
| 7     | [07 — MoveIt runtime parity](07-moveit-runtime-parity.md)                       | M    | Existing MoveIt smoke/runtime tests run with unchanged assertions.              |
| 8     | [08 — Optional CI and metrics](08-optional-ci-and-metrics.md)                   | M    | Advisory dual-architecture jobs, artifacts, failure drills, and metrics.        |
| 9     | [09 — Observation and rollout decision](09-observation-and-rollout-decision.md) | S    | Twenty-build ledger, rollback drill, and approved rollout decision.             |

Dependency sketch:

```text
01 -> 02 -> 03 -> 04 -> 05 -> 06 -> 07 -> 08 -> 09
```

The order is intentionally linear. Later specs extend the same Bazel runtime
closure and would otherwise create overlapping BUILD, dependency, and CI
changes that are difficult for independent agents to reconcile.

## Package ownership

This table prevents an agent from widening its PR while chasing dependencies.
"Host boundary" means the package may be consumed from the pinned Jazzy
development image but is not claimed as Bazel-built or hermetic.

| Package                              | Owning spec | Migration treatment                                                                        |
| ------------------------------------ | ----------- | ------------------------------------------------------------------------------------------ |
| `drqp_interfaces`                    | 02          | Bazel-built ROSIDL package.                                                                |
| `drqp_rapidjson`                     | 04          | Header-only Bazel library from the checked-in headers.                                     |
| `drqp_serial`                        | 04          | Bazel-built C++ library and focused tests.                                                 |
| `drqp_a1_16_driver`                  | 04          | Bazel-built C++ library and focused tests.                                                 |
| `drqp_control`                       | 04          | Bazel-built libraries, executable, plugin, data, and focused tests.                        |
| `drqp_joy`                           | 05          | Bazel-built component/node and focused test.                                               |
| `drqp_kinematics`                    | 05          | Bazel Python library and non-lint pytest suite.                                            |
| `drqp_brain`                         | 05          | Bazel Python library, four entry points, data, and focused tests.                          |
| `drqp_launch_testing`                | 05          | Bazel Python library and helper tests.                                                     |
| `drqp_keyboard_control`              | 05          | Bazel Python library, entry point, and headless tests.                                     |
| `launch`, `launch_pytest` (vendored) | 05          | Preserve repository patches; do not replace with the rules fork copy without parity proof. |
| `sdl3_vendor`                        | 05          | Build the SDL 3.4.4 commit already pinned by CMake.                                        |
| `drqp_gazebo`                        | 06          | Bazel launch/data/test targets; Gazebo binaries are a host boundary.                       |
| `drqp_moveit`                        | 07          | Bazel launch/data/test targets; MoveIt binaries/modules are a host boundary.               |
| `drqp_lint_common`                   | none        | Remains part of the colcon lint oracle; do not make runtime targets depend on it.          |
| `drqp_robot_mcp`                     | deferred    | Explicitly outside the representative rollout surface.                                     |

## Contract for implementing agents

Before changing files, an agent must confirm that the dependency spec is
merged into its base. If it is not, stop and report the missing prerequisite;
do not recreate or cherry-pick the predecessor inside the current PR.

Every spec uses this delivery contract:

1. Add tests or analysis targets first and show the missing/failing behavior.
2. Implement only the packages and shared Bazel helpers assigned to the spec.
3. Keep `package.xml`, CMake/setup metadata, source lists, data lists, compiler
   standards, warnings, and test assertions unchanged unless the spec names a
   required parity correction. A correction must apply to both build paths.
4. Run the exact Bazel and colcon commands listed by the spec. ROS/colcon
   commands must use `scripts/with-ros-env.sh`. Bazel commands use the
   repository-owned `scripts/bazel.sh`; host-runtime tests may wrap it with
   `scripts/with-ros-env.sh` only through the explicit `host-ros` config.
5. Create `docs/agents/specs/bazel-ros2-migration/evidence/NN-<slug>.md` with:
   the rules SHA, Bazel/Python/ROS versions, architecture, exact commands and
   exit codes, tests run, host-supplied dependencies, known non-hermetic
   inputs, deviations from the public label contract, and remaining blockers.
6. Update the owning spec's status only in the landing PR. A failing mandatory
   acceptance item leaves the spec proposed or blocked; skips are not passes.

For a failed build or test, include the smallest reproducer and the relevant
Bazel test log or `log/latest_{build,test}` package log in the handoff. Never
weaken sandboxing, warnings, launch exit checks, assertions, or retry semantics
to turn an infrastructure failure green.

## Bazel conventions fixed by this program

- Bazel packages mirror the existing ROS package directories.
- Public first-party target names are fixed in each owning spec. Internal
  helpers may vary, but downstream specs must not need to guess labels.
- Use explicit `srcs`, `hdrs`, `deps`, and `data`; do not use recursive globs.
- Bazel may read checked-in sources and resources only. It must not consume
  this workspace's `build/`, `install/`, `log/`, `.venv/`, or generated docs.
- A host ROS boundary is allowed only under `--config=host-ros`, must resolve
  to `/opt/ros/jazzy` in the pinned development image, and must never resolve
  to this workspace's `install/` tree.
- Tests that communicate over DDS or start Gazebo/MoveIt must be tagged
  `requires-network`, run without remote execution unless proven compatible,
  and use isolated ROS/Gazebo domains. Unit tests remain network-sandboxed.
- `MODULE.bazel.lock` and every requirements lock/integrity value are reviewed
  inputs. Mutable branches, unverified archives, and floating pip ranges are
  not allowed in the Bazel dependency graph.
- Ament/colcon remains an independent oracle. A Bazel target must not replace,
  disable, or conditionally skip its existing ament test registration.

## Global promotion gates

Bazel may become required CI only after all are true:

- [ ] The rules fork and fetched dependencies are immutable and provenance is
      recorded.
- [ ] A clean checkout passes the Spec 02–07 selected target set without local
      colcon artifacts.
- [ ] All five custom messages pass C++, Python, and cross-language checks.
- [ ] Pluginlib loads `drqp_control/a1_16_hardware_interface` from Bazel
      runtime data.
- [ ] The selected Python, control, Gazebo, and MoveIt tests retain their
      behavioral assertions and per-process exit-code checks.
- [ ] Host-supplied Gazebo/MoveIt dependencies are versioned and plainly
      reported; no report calls the build hermetic while they remain.
- [ ] Colcon and Bazel pass independently on the same commit for 20
      consecutive default-branch builds after the full selected CI set lands.
- [ ] Clean/no-op build time, focused-test time, local cache behavior, disk
      use, setup cost, and failure diagnosis are measured on both CI
      architectures.
- [ ] Disabling the Bazel job leaves the existing colcon release path intact.
- [ ] Maintainers record one decision: `promote`, `retain-optional`, or `stop`.

## Explicit non-goals

- Removing ament metadata, colcon, or release jobs during this program.
- Building all 17 ROS packages or claiming `//...` support.
- Replacing ROS middleware or ament-index conventions.
- Treating simulation/build parity as hardware safety validation.
- Migrating `drqp_robot_mcp`, repository lint tooling, docs, or notebooks.
- Enabling remote execution or a remote cache before local correctness is
  established.

## Verified upstream sources

- ROS 2, [ament build-system design](https://design.ros2.org/articles/ament.html).
- `mvukov/rules_ros2`, [Jazzy support PR #558](https://github.com/mvukov/rules_ros2/pull/558).
- `mvukov/rules_ros2`, [OMG IDL support PR #617](https://github.com/mvukov/rules_ros2/pull/617).
- Candidate upstream branch head on 2026-07-15:
  `56ad1dfa6a636378e623dd5903c4ff4d7d2acd4b`.
