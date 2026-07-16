# Spec 08: Optional CI and metrics

- **Status**: proposed
- **Depends on**: [Spec 07](07-moveit-runtime-parity.md)
- **Files**: GitHub Actions, Bazel CI scripts/config, migration evidence
- **Size**: M

## Objective

Add an advisory Bazel CI path on the existing x86_64/aarch64 Jazzy development
images, publish test/diagnostic artifacts, and define reproducible performance
measurements. Keep the colcon job required and unchanged.

## CI placement and controls

Extend `.github/workflows/ci.yml` with Bazel jobs that consume the same pinned
`merge-dev-image.outputs.image_pinned` as `ros-ci`. Do not duplicate development
image construction or install dependencies ad hoc in the test job.

Use repository variables with these exact semantics:

- `BAZEL_CI_ENABLED`: job runs only when value is `1`; any other value is the
  one-click rollback/disable path.
- existing `AMD_ONLY`/`ARM_ONLY`: preserve current architecture selection.

Creating/changing repository variables is an explicit repository-owner action.
An implementation agent without that authority must land the workflow with the
safe disabled default, record the requested values, and stop before claiming a
manual CI run or completed acceptance gate.

Workflow dispatch with `force_run` must still be able to exercise the Bazel job
when enabled. Add Bazel/module/BUILD/evidence paths to the existing path filter
so relevant changes cannot skip the job.

Do not modify the colcon `ros-ci` commands, coverage publication, image tags,
or release behavior.

## Job split

### `bazel-core`

Run on both current architectures with a 45-minute initial timeout:

```text
//packages/runtime/drqp_interfaces:interface_tests
//packages/runtime/drqp_control:control_tests
//packages/runtime/drqp_control:control_launch_test
//packages/runtime/drqp_launch_testing:launch_testing_tests
//packages/runtime/drqp_kinematics:kinematics_tests
//packages/runtime/drqp_joy:joy_tests
//packages/runtime/drqp_brain:brain_tests
//packages/simulation/drqp_keyboard_control:keyboard_control_tests
```

This job does not select `host-ros` and must remain independent of
`/opt/ros/jazzy` runtime libraries.

### `bazel-runtime`

Run with `scripts/with-ros-env.sh` and `--config=host-ros` on both current
architectures, initially with a 60-minute timeout:

```text
//packages/simulation/drqp_gazebo:gazebo_smoke_tests
//packages/runtime/drqp_moveit:host_moveit_compatibility_test
//packages/runtime/drqp_moveit:moveit_smoke_tests
```

### `bazel-runtime-full`

Run the Gazebo full group and MoveIt runtime test on default-branch pushes and
manual dispatch, not on every pull request:

```text
//packages/simulation/drqp_gazebo:gazebo_full_tests
//packages/runtime/drqp_moveit:moveit_runtime_test
```

Use a 90-minute initial timeout. This job is part of the observation gate even
though it is omitted from ordinary PRs.

All three jobs are advisory because branch protection does not require their
checks in this spec. A failing Bazel step and job must still produce a failed
conclusion visible in the UI and artifacts; do not hide failures with
`continue-on-error` or `|| true`.

## CI runner script

Add `scripts/bazel-ci.sh` as a thin, non-interactive orchestrator over
`scripts/bazel.sh`. It must:

- accept a named target group (`core`, `runtime`, or `runtime-full`);
- use a repository-relative `./.tmp/bazel-ci/<arch>` output user root, never a
  system temporary-directory environment variable;
- emit a Bazel execution profile and build-event JSON for every invocation;
- preserve Bazel's exit code while retaining logs/artifacts;
- print the rules SHA, Bazel/Python/ROS version, architecture, image digest,
  selected labels, elapsed time, and disk consumption;
- never source a workspace colcon overlay or install dependencies.

The host runtime groups must be invoked through `scripts/with-ros-env.sh`; the
core group invokes `scripts/bazel-ci.sh` directly.

## Required measurements

Capture these fields in a machine-readable CI summary and human-readable job
summary:

| Metric            | Definition                                                                                                            |
| ----------------- | --------------------------------------------------------------------------------------------------------------------- |
| clean build/test  | First invocation in an empty repository-local output user root.                                                       |
| no-op             | Immediate second invocation of the same target group with no changes.                                                 |
| focused test      | Per-label duration from Bazel test XML/profile.                                                                       |
| cache             | Local action cache hits/misses from profile/BEP; remote cache fields are `not configured` until deliberately enabled. |
| disk              | Output user root size after first and second invocation.                                                              |
| dependency fetch  | Time and bytes attributable to external repository resolution when available.                                         |
| failure diagnosis | Log/artifact path and elapsed time to identify one induced failure from the implementation PR.                        |

Do not compare a Bazel clean build to an incremental colcon build. For cost
reporting, compare clean-to-clean and no-op-to-no-op on the same selected
package graph; label the existing full-workspace colcon timing separately.

## Artifacts and summaries

Always upload, including on cancellation/failure when files exist:

- Bazel `testlogs/**/test.xml`, `test.log`, and undeclared-output archives;
- execution profiles and build-event JSON;
- the generated metrics summary;
- wrapper stdout/stderr and resolved host-package version list for runtime
  jobs.

Use architecture/job-qualified artifact names so matrix runs do not overwrite
one another. Add a concise GitHub job summary with passed/failed target counts,
duration, cache data, disk use, rules SHA, and the non-hermetic host-boundary
notice.

## Tests to add first

- A shell test for target-group expansion, unknown-group failure, exit-code
  preservation, `.tmp` output-root use, and summary fields.
- A workflow/static validation that all public labels from Specs 02–07 appear
  in exactly one intended CI group and that the colcon job text is unchanged.
- An implementation-PR failure drill in a throwaway commit/branch: invalid
  message definition for core, missing Gazebo data for runtime, and invalid
  MoveIt plugin/config for runtime-full. Revert each change after capturing the
  deterministic failing target and artifact path.

Do not commit intentionally broken inputs.

## Verification

```bash
scripts/bazel.sh test //bazel/tests:all
scripts/bazel-ci.sh core
scripts/with-ros-env.sh scripts/bazel-ci.sh runtime
scripts/with-ros-env.sh scripts/bazel-ci.sh runtime-full
```

Validate the workflow syntax with the repository's pinned actionlint/pre-commit
tooling, then trigger one manual advisory run on both architectures. Record run
links, check names, artifact names, timings, and the failure drill in
`evidence/08-optional-ci.md`.

## Allowed files

- `.github/workflows/ci.yml` and the existing path-filter input needed to route
  Bazel changes;
- `scripts/bazel-ci.sh`, Bazel policy tests/config, and CI artifact helpers;
- this program's evidence/status documentation.

Do not modify package sources/tests, the colcon job body, release workflows,
branch protection, or required-check policy in this spec.

## Acceptance criteria

- [ ] Core, runtime-smoke, and default-branch full target groups are explicit
      and run on both supported CI architectures.
- [ ] `BAZEL_CI_ENABLED` disables only Bazel jobs; colcon remains unchanged.
- [ ] Bazel failures have failed conclusions but remain advisory because the
      checks are not required by branch protection.
- [ ] Test XML/logs, profiles, BEP, metrics, and host-version artifacts publish
      on success and failure with unique names.
- [ ] Clean/no-op/cache/disk/focused-test measurements use comparable
      definitions and repository-relative `.tmp` storage.
- [ ] The induced failures are actionable and stale output does not mask them.
- [ ] `evidence/08-optional-ci.md` contains the required handoff data.
