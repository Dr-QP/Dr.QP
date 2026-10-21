---
type: codebase
description: How a pull request is validated — primary-checks fan-out, dev-image build, colcon build/test in two containers, slow simulation tier, deploy image.
source: .github/workflows
source_digest: sha256:9451fb5b54609479b4f191d48cba33ead37600983fe8e9be00dc6faa7d00553b
verified:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
stale_after: 2027-01-19
generated:
  by: claude-code/opus-5.5
  at: 2026-10-21T01:35:27Z
sources:
- id: code
  resource: .github/workflows
---

# flow-ci

The build-and-test path from a push or pull request to promoted desktop and
deploy images.

## Trace

1. **Trigger.** `primary-checks.yml` runs on push, pull request, merge queue, or
   dispatch. It calls `reformat`, `ci`, `docs`, and `codeql`. On `main` a new
   run queues behind the running one; elsewhere it cancels it.
   ([github/workflows](github/workflows.md), `primary-checks.yml:56`)

2. **Dev image.** `ci.yml` filters on paths — a push to `main` against the last
   commit this workflow passed on — then builds
   `docker/ros/desktop/ros-desktop.Dockerfile` for each architecture and merges
   the result into `<distro>-ros-desktop` under its `sha-<commit>` candidate tag
   only. ([docker/ros](docker/ros.md), `ci.yml:65`)

3. **Build.** `ros-ci` runs in the dev image:

   - `scripts/ros-dep.sh`, then `source scripts/setup.bash`.
   - `colcon build --mixin coverage-pytest ninja rel-with-deb-info --symlink-install --cmake-args -D DRQP_ENABLE_COVERAGE=ON`.

   ([scripts](scripts.md), `ci.yml:178`)

4. **Test.** `colcon test --return-code-on-test-failure` with `-rA`, then LLVM
   coverage export (`packages/cmake/llvm-cov-export-all.py`) and a Codecov
   upload. The Gazebo suite runs only its smoke tier here. (`ci.yml:195`)

5. **Devcontainer test.** `dev-container-ci` repeats the build and test inside
   the devcontainer. One matrix entry sets `DRQP_TEST_MODE=slow` to add the full
   [drqp_gazebo](packages/simulation/drqp_gazebo.md) tier, with a 60-minute
   timeout. (`ci.yml:225`)

6. **Deploy image.** After `ros-ci` passes, `build-deploy-image` builds
   `ros-deploy.Dockerfile` for each architecture and merges it into
   `<distro>-ros-deploy`, again under its candidate tag only. (`ci.yml:336`)

7. **Promote.** Once `ros-ci` and `dev-container-ci` pass, `promote-images`
   points the event's moving tags (`edge`, branch, `pr-N`, semver) of both
   images at the tested digests. `merge-all` then gates completion.
   (`ci.yml:407`)

## Failure modes

- A failing test shows up only as the failing package name in the job log. The
  failure itself is in the `colcon-logs-<arch>` and `colcon-test-reports-<arch>`
  artifacts.
- Any failed test job leaves `edge` and the other moving tags on the last
  promoted image; only the `sha-<commit>` candidate tag names the new one.
- The slow Gazebo tests can wedge. Each has an 1800 s ctest timeout so that one
  test cannot use up the job's budget.
