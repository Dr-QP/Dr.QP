# Spec 01: Fork and Bazel foundation

- **Status**: proposed
- **Depends on**: none
- **Packages**: repository tooling only
- **Size**: M

## Objective

Create the immutable `rules_ros2` supply-chain boundary and a reproducible,
optional Bazel 8.7.0 entry point. This PR establishes configuration and
isolation; it does not migrate a ROS package.

## Preconditions and stop conditions

- The target fork is `https://github.com/Dr-QP/rules_ros2`.
- The tested upstream candidate is
  `mvukov/rules_ros2@56ad1dfa6a636378e623dd5903c4ff4d7d2acd4b`.
- If the fork does not exist and the agent cannot create it in the Dr-QP
  organization, stop before editing this workspace and report the required
  owner action. Do not pin directly to a mutable upstream branch as a fallback.
- If upstream commit `56ad1dfa...` cannot complete its own documented bzlmod
  smoke test with Bazel 8.7.0 on x86_64, record the reproducer and stop. A local
  fork patch belongs in a reviewed fork commit before this workspace pins it.

## Required fork state

Create or verify the `Dr-QP/rules_ros2` fork, then mirror the candidate through
a local git fetch/push workflow. Do not use a GitHub API to update branch refs.

The fork must contain:

- an immutable full commit reachable from a protected `jazzy` branch or signed
  `jazzy-YYYYMMDD` tag;
- the upstream Apache-2.0 `LICENSE` unchanged;
- `UPSTREAM.md` recording the upstream URL, source SHA, retrieval date, Bazel
  version, sync commands, and a table of local patches with rationale and
  upstream issue/PR links;
- no force-push requirement for the ref consumed by this workspace.

If no fork patches are needed, the fork commit may equal the upstream candidate
SHA. If patches are needed, the evidence file must record both SHAs.

## Workspace deliverables

### Dependency and toolchain files

- `.bazelversion`: exactly `8.7.0`, matching the candidate rules revision.
- `MODULE.bazel`: declare the workspace module, depend on
  `com_github_mvukov_rules_ros2`, and override it with the immutable Dr-QP fork
  archive. Pin the archive by full revision and `integrity`; do not use
  `local_path_override`, a branch archive, or an unverified URL.
- `MODULE.bazel.lock`: generated from a clean dependency resolution and
  committed.
- Python toolchain: exactly 3.12, matching Jazzy and the rules fork's pip lock.
  Do not add an independent floating pip resolve in this spec.
- Import only the non-module ROS repositories needed by Spec 02:
  `ros2_common_interfaces`, `ros2_rcl`, `ros2_rclcpp`, `ros2_rclpy`,
  `ros2_rosidl`, `ros2_rosidl_python`, `ros2_rosidl_runtime_py`, and their
  required transitive repositories. Do not eagerly expose Gazebo or MoveIt.

### Repository configuration

- `.bazelrc`: carry forward the candidate fork's mandatory bzlmod, explicit
  Python-init, C++17 minimum, strict-action-environment, and network-sandbox
  settings. Keep `--test_output=errors` and import ignored local overrides only
  from `user.bazelrc`.
- Define `build:host-ros` and `test:host-ros` configs, but keep them unused
  until Spec 06. They may expose only `/opt/ros/jazzy` from the pinned
  development image and must be visibly named non-hermetic.
- `.bazelignore`: include `build`, `install`, `log`, `.venv`, `.tmp`, and
  `docs/_build`; also ignore any Bazel output symlink created at repository
  root by the chosen symlink-prefix configuration.
- `.gitignore`: ignore Bazel output symlinks, a repository-local output root if
  used, and `user.bazelrc`; do not ignore `MODULE.bazel.lock`.
- `bazel/README.md`: document prerequisites, supported architectures,
  dependency-update procedure, target-label conventions, host-boundary policy,
  clean/reset commands, and where Bazel test logs live.

### Entry point

Add executable `scripts/bazel.sh` as the only documented Bazel entry point.
It must:

1. locate the repository root from its own path and work from any caller CWD;
2. use Bazelisk or Bazel 8.7.0 and fail with an actionable install/version
   diagnostic otherwise;
3. set no home-directory-specific paths and preserve the caller's arguments and
   exit code;
4. never source `scripts/setup.bash`, `install/setup.bash`, or
   `install/local_setup.bash`;
5. reject any environment value that points at this workspace's `build/`,
   `install/`, or `log/` through `AMENT_PREFIX_PATH`, `CMAKE_PREFIX_PATH`,
   `COLCON_PREFIX_PATH`, `PYTHONPATH`, `LD_LIBRARY_PATH`, or `PATH`;
6. permit `/opt/ros/jazzy` only when the caller explicitly selects
   `--config=host-ros`.

Do not change `scripts/with-ros-env.sh`, `scripts/setup.bash`, existing colcon
commands, or current workflows in this spec.

## Tests to add first

Add a small shell test target named `//bazel/tests:workspace_isolation_test`.
The test must run the entry point with synthetic forbidden environment values
and assert rejection, then verify a clean environment reaches Bazel. It must
not depend on ROS installation or network after external repositories resolve.

Also add a repository check target named `//bazel/tests:dependency_policy_test`
that fails when:

- the rules override is not a full immutable revision with integrity;
- `MODULE.bazel.lock` is absent;
- a forbidden generated directory is not ignored; or
- a mutable GitHub branch archive appears in `MODULE.bazel`.

Keep these policy tests narrow; do not write a general BUILD-file linter.

## Verification

Run from a clean checkout with no workspace `build/`, `install/`, or `log/`:

```bash
scripts/bazel.sh version
scripts/bazel.sh mod graph
scripts/bazel.sh test //bazel/tests:all
scripts/bazel.sh query '@com_github_mvukov_rules_ros2//ros2:all'
scripts/with-ros-env.sh colcon list --names-only
```

Then run `scripts/bazel.sh clean --expunge`, repeat the policy tests, create a
normal colcon overlay, confirm the wrapper rejects that contaminated
environment, remove/unset the overlay variables, and confirm the tests pass.

Record the fork SHA, archive integrity, `bazel mod graph` summary, and commands
in `evidence/01-foundation.md`.

## Allowed files

- Fork repository: `UPSTREAM.md` and narrowly required fork fixes.
- This repository: `.bazelversion`, `.bazelrc`, `.bazelignore`, `.gitignore`,
  `MODULE.bazel`, `MODULE.bazel.lock`, `bazel/**`, `scripts/bazel.sh`, and this
  program's evidence/status documentation.

Do not add BUILD files below `packages/`; Spec 02 owns the first ROS package.

## Acceptance criteria

- [ ] The Dr-QP fork, upstream SHA, consumed fork SHA, license, and patch policy
      are recorded and immutable.
- [ ] Bazel 8.7.0 and Python 3.12 are pinned; module resolution is locked.
- [ ] `scripts/bazel.sh` works from outside the repository root and provides
      actionable missing-tool/version diagnostics.
- [ ] Both `//bazel/tests:all` policy tests pass after a clean expunge.
- [ ] Workspace colcon output is rejected rather than silently consumed.
- [ ] Existing colcon scripts and workflows are unchanged and package discovery
      still lists all 17 packages.
- [ ] `evidence/01-foundation.md` contains the required handoff data.
