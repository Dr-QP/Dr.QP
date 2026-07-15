# Spec 01: Fork and Bazel foundation

- **Status**: proposed
- **Depends on**: none
- **Size**: M

## Objective

Create a reproducible, optional Bazel entry point based on a project-owned
fork of the Jazzy implementation in `mvukov/rules_ros2` PR #558. It must not
change the existing colcon release path.

## Fork contract

1. Create a fork in the project's GitHub organization from `mvukov/rules_ros2`.
2. Create a branch named `jazzy` from the exact tested head of upstream
   `feature/jazzy`; record the upstream repository URL, branch, full commit
   SHA, and retrieval date in this specification or a machine-readable lock.
3. Add `UPSTREAM.md` in the fork with the upstream URL, original license,
   attribution, synchronization procedure, and every local patch with its
   rationale and upstream issue/PR link.
4. Do not force-push the project `jazzy` branch after it is consumed. Promote
   changes through reviewed commits or release tags.
5. Prefer upstream contributions for generic rules_ros2 fixes. Keep local
   changes narrowly scoped to workspace compatibility or unreleased upstream
   fixes.

## Repository design

- Add Bazel version selection and bzlmod configuration (`.bazelversion` and
  `MODULE.bazel`), pinning all external repositories and archives. Select the
  Bazel version supported by the pinned rules fork; document why.
- Fetch the rules fork by immutable commit or signed project tag, never a
  mutable branch name. Use integrity hashes for archive downloads.
- Add a top-level `.bazelrc` with reproducible defaults. Do not hard-code a
  developer home directory, `ROS_DISTRO` installation, or generated workspace
  paths.
- Add `.bazelignore` entries for `build/`, `install/`, `log/`, `.venv/`, and
  other generated directories so Bazel neither indexes nor consumes colcon
  output.
- Add `scripts/bazel.sh` as the one documented entry point. It must set only
  repository-local configuration and pass its arguments unchanged to Bazel.
  It must not source `install/setup.bash` or use colcon artifacts.
- Keep `scripts/with-ros-env.sh` and all current colcon workflows unchanged.

## First targets

Initially create only the target graph necessary to build and test
`drqp_interfaces` and `drqp_control`, plus their direct first-party
dependencies. Do not claim `//...` support until Spec 03's expanded runtime
set passes.

Use a target naming and visibility convention that maps one ROS package to one
Bazel package where practical. Keep non-generated source and resource files as
declared Bazel inputs; do not copy them into source-controlled generated paths.

## Test plan

- In a clean clone with no `build/`, `install/`, or `log/` directories, run
  `scripts/bazel.sh build` on the initial target set.
- Repeat after `bazel clean --expunge` and verify the command has no dependency
  on a sourced colcon overlay.
- Run the same command after a normal colcon build, then delete colcon outputs
  and repeat; the Bazel action graph and result must remain valid.
- Validate repository and fork pinning with Bazel's module/dependency tooling.
- Verify `scripts/bazel.sh` documents required host prerequisites and fails
  with an actionable diagnostic when they are absent.

## Acceptance criteria

- [ ] The project-owned fork, immutable Jazzy revision, provenance, license,
      and local-patch policy are recorded.
- [ ] `MODULE.bazel`, `.bazelversion`, `.bazelrc`, and `.bazelignore` provide
      a reproducible optional Bazel configuration.
- [ ] Initial builds neither source nor read colcon output directories.
- [ ] Existing colcon build, test, formatting, and release commands are
      unaffected.
- [ ] The initial Bazel graph is limited to the Spec 02 proof surface and
      its dependencies.
