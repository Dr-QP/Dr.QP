# Spec 09: Observation and rollout decision

- **Status**: proposed
- **Depends on**: [Spec 08](08-optional-ci-and-metrics.md) and 20 qualifying
  default-branch builds
- **Files**: migration evidence/decision documentation; CI policy only after
  maintainer approval
- **Size**: S implementation, time-gated observation

## Objective

Audit an uninterrupted observation window, compare correctness and operating
cost with colcon, exercise rollback, and record one explicit decision:
`promote`, `retain-optional`, or `stop`.

This spec cannot be completed in the same PR that introduces optional CI. Do
not fabricate or extrapolate the 20-build record.

## Qualifying-build definition

A default-branch commit qualifies only when the workflow run for that commit
has all of the following final results on both configured architectures:

- existing colcon `ros-ci`: success;
- `bazel-core`: success;
- `bazel-runtime`: success;
- `bazel-runtime-full`: success;
- required test/log/metric artifacts: present and readable.

A cancelled or skipped Bazel job does not qualify. A product/test failure
resets the consecutive count to zero. A documented runner/service outage may be
rerun on the same commit; record the original and rerun and count that commit
once only if the rerun succeeds without code/config changes.

Reset the count after any change to:

- the rules fork revision or Bazel/module/Python dependency locks;
- the CI target groups or host development image;
- test assertions, fixture/retry behavior, or selected package graph;
- sandbox, host-boundary, or cache configuration.

Ordinary source changes do not reset the streak when all qualifying jobs pass;
exercising normal change is the point of the window.

## Observation ledger

Create `evidence/09-observation-ledger.md` with one row per default-branch
commit:

| Field          | Required value                                              |
| -------------- | ----------------------------------------------------------- |
| sequence       | streak number or `reset`                                    |
| commit         | full SHA and date                                           |
| workflow       | run URL and attempt                                         |
| image/rules    | image digest and rules SHA                                  |
| colcon         | x86_64/aarch64 result and duration                          |
| Bazel core     | results, clean/no-op time, cache, disk                      |
| Bazel runtime  | results and duration                                        |
| Bazel full     | results and duration                                        |
| artifacts      | links/names or missing reason                               |
| classification | pass, product failure, flaky test, infra failure, cancelled |
| action         | issue/PR/retry/reset reference                              |

Use workflow evidence, not recollection. Investigate every non-pass and link
the remediation; do not relabel deterministic product failures as infra.

## Cost and operability report

At 20 qualifying builds, aggregate median and p95 for both architectures:

- clean and no-op selected-graph duration;
- focused test and full runtime duration;
- local cache hit/miss rates and disk use;
- dependency-fetch failures/time;
- total additional runner minutes and artifact storage;
- failure-diagnosis examples and maintainer intervention count;
- clean developer setup on one documented machine or devcontainer.

Compare like-for-like selected graphs where possible and separately report the
existing full colcon workspace. List all non-hermetic host dependencies and any
rules-fork patches still not accepted upstream.

## Rollback drill

With maintainer approval, set `BAZEL_CI_ENABLED` away from `1` for one manual
workflow dispatch or controlled default-branch run. Verify:

- all Bazel jobs are skipped/disabled as designed;
- colcon build/test, coverage, image, and release behavior still run normally;
- no production source, ament metadata, or artifact conversion is required;
- re-enabling the variable restores Bazel jobs without a code change.

Record before/disabled/re-enabled runs. This drill does not count toward the
20-build streak and must not be performed by an agent without repository-owner
authorization.

## Decision record

Create `docs/agents/specs/bazel-ros2-migration/migration-decision.md` containing:

1. decision and date;
2. selected target/package scope;
3. correctness and observation evidence;
4. cost/operability summary by architecture;
5. hermetic and host-supplied boundaries;
6. fork ownership/upstream patch burden;
7. known gaps, including deferred `drqp_robot_mcp` and no `//...` claim;
8. rollback drill result;
9. approver names/links and follow-up owners.

Decision effects:

- `promote`: with maintainer approval, require the PR-capable `bazel-core` and
  `bazel-runtime` checks in branch protection. Keep colcon required and keep
  `bazel-runtime-full` on default branch/manual runs.
- `retain-optional`: leave Bazel advisory/enabled, state the next review trigger
  and owner, and keep colcon required.
- `stop`: set `BAZEL_CI_ENABLED` away from `1`, keep the colcon path intact, and
  open a separate cleanup issue. Do not delete Bazel/source files in this
  decision PR.

Branch-protection/repository-variable changes require explicit maintainer
authorization and must be recorded. Do not use a GitHub API to update branch
refs or push contents.

## Verification

- Independently recount the ledger from workflow runs and confirm exactly 20
  uninterrupted qualifying commits after the last reset.
- Verify every ledger artifact link/name and recompute aggregate metrics from
  source summaries.
- Run the rollback drill and one post-restore advisory run.
- Review the decision record against every global promotion gate in README.

## Allowed files

- `evidence/09-observation-ledger.md`, `migration-decision.md`, and this
  program's status documentation;
- repository variables/check policy only when the approved decision requires
  it.

Do not modify package sources, BUILD targets, dependency locks, tests, or the
colcon workflow in this spec; any such change invalidates the observation
window and belongs in a predecessor follow-up.

## Acceptance criteria

- [ ] The ledger contains 20 consecutive qualifying default-branch commits
      after the last dependency/CI reset, with evidence for both architectures.
- [ ] Every failure/cancellation is classified and resets or reruns the streak
      according to the written rules.
- [ ] Cost, cache, disk, setup, flake, diagnosis, host-boundary, and fork burden
      are aggregated and reviewed.
- [ ] The authorized rollback drill proves colcon/release independence and
      successful Bazel re-enable.
- [ ] `migration-decision.md` records `promote`, `retain-optional`, or `stop`
      with maintainer approval and explicit consequences.
- [ ] Any CI/branch-policy change matches the decision; colcon remains required.
