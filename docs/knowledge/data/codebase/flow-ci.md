---
type: codebase
description: How a pull request is validated — primary-checks fan-out, dev-image build, colcon build/test in two containers, slow simulation tier, deploy image.
source: .github/workflows
source_digest: sha256:1d9fa812c20df7a86d1dbf94475607e12f46faca477ddfe330aa161235f3499e
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-11-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: .github/workflows
---

# flow-ci

The build-and-test path from a push or pull request to a published deploy image.

## Trace

1. **Trigger.** `primary-checks.yml` runs on push, pull request, merge queue, or
   dispatch. It calls `reformat`, `ci`, `docs`, and `codeql`.
   ([github/workflows](github/workflows.md), `primary-checks.yml:46`)

2. **Dev image.** `ci.yml` filters on paths, then builds
   `docker/ros/desktop/ros-desktop.Dockerfile` for each architecture and merges
   the result into `<distro>-ros-desktop`. ([docker/ros](docker/ros.md),
   `ci.yml:64`)

3. **Build.** `ros-ci` runs in the dev image:

   - `scripts/ros-dep.sh`, then `source scripts/setup.bash`.
   - `colcon build --mixin coverage-pytest ninja rel-with-deb-info --symlink-install`.

   ([scripts](scripts.md), `ci.yml:176`)

4. **Test.** `colcon test --return-code-on-test-failure` with `-rA`, then LLVM
   coverage export (`packages/cmake/llvm-cov-export-all.py`) and a Codecov
   upload. The Gazebo suite runs only its smoke tier here. (`ci.yml:193`)

5. **Devcontainer test.** `dev-container-ci` repeats the build and test inside
   the devcontainer. One matrix entry sets `DRQP_TEST_MODE=slow` to add the full
   [drqp_gazebo](packages/simulation/drqp_gazebo.md) tier, with a 60-minute
   timeout. (`ci.yml:223`)

6. **Deploy image.** After `ros-ci` passes, `build-deploy-image` builds
   `ros-deploy.Dockerfile` for each architecture and merges it into
   `<distro>-ros-deploy`. `merge-all` then gates completion. (`ci.yml:334`)

## Failure modes

- A failing test shows up only as the failing package name in the job log. The
  failure itself is in the `colcon-logs-<arch>` and `colcon-test-reports-<arch>`
  artifacts.
- The slow Gazebo tests can wedge. Each has an 1800 s ctest timeout so that one
  test cannot use up the job's budget.
