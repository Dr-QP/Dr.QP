---
type: codebase
description: Reusable launch_pytest helpers that restore per-process exit-code checking, plus a test suite proving launch_pytest shutdown behavior.
source: packages/runtime/drqp_launch_testing
source_digest: sha256:15bb8d81ee731f9896303d0973debddb671c49c9f1cb2e2d0ee876882e1561eb
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T11:05:15Z
sources:
- id: code
  resource: packages/runtime/drqp_launch_testing
---

# drqp_launch_testing

"Reusable launch_pytest helpers for Dr.QP, including per-process exit-code
verification." In `launch_pytest`, `shutdown=True` tests assert only the
aggregate launch-service return code. This package brings back the per-process
check.

## Public surface

- `track_process_exit_codes(launch_description)` returns a `ProcInfoHandler`
  that records each process's exit.
- `assert_processes_exited_cleanly(proc_info, ignore=...)`.
- `DEFAULT_SHUTDOWN_KILLED_PROCESSES` is the default ignore list:
  - `gazebo`, `gz`
  - `bridge_node`
  - `move_group`
  - `spawner`
  - `drqp_brain`
  - `drqp_robot_state`

## How it works

Tracking registers an `OnProcessExit` handler on the launch description. The
assertion uses `launch_testing.asserts` and skips processes whose name contains
an ignored token.

`test/shutdown_behavior/` holds six fixture-scope and retry "combos" and a
`SPEC.md` recording how `launch_pytest` shutdown and `pytest-retry` interact
with the vendored `launch_pytest`.

## Depends on

- `launch`, `launch_testing`
- The vendored `launch_pytest` in `packages/vendor/launch`

## Invariants & gotchas

- Tokens match by substring, so they must stay specific. `drqp_robot_state` must
  not become `robot_state`, which would also hide `robot_state_publisher`.
- `drqp_brain` (SIGABRT) and `drqp_robot_state` (take_message error) are on the
  ignore list because they die in rclpy teardown often enough; no reliable fix
  has been found.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_launch_testing/drqp_launch_testing/process_exit_codes.py:62`
  — `DEFAULT_SHUTDOWN_KILLED_PROCESSES`
- `packages/runtime/drqp_launch_testing/drqp_launch_testing/process_exit_codes.py:73`
  — `track_process_exit_codes`
- `packages/runtime/drqp_launch_testing/drqp_launch_testing/process_exit_codes.py:91`
  — `assert_processes_exited_cleanly`
- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/SPEC.md` —
  shutdown and retry matrix
