---
type: bug
description: drqp_joystick_translator sometimes dies from SIGINT (exit code -2) when a launch test tears down, failing the move_group smoke test's clean-exit check.
generated:
  by: claude-code/opus-5.5
  at: 2026-10-13T07:12:10Z
sources:
- id: test
  resource: packages/runtime/drqp_moveit/test/test_move_group_launch_smoke.py
- id: translator
  resource: packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py
- id: exit-codes
  resource: packages/runtime/drqp_launch_testing/drqp_launch_testing/process_exit_codes.py
- id: bringup
  resource: packages/runtime/drqp_brain/launch/bringup.launch.py
- id: ci-run
  resource: https://github.com/Dr-QP/Dr.QP/actions/runs/38319897008
- id: pr-561
  resource: https://github.com/Dr-QP/Dr.QP/pull/561
---

# Bug: joystick translator exits on SIGINT in launch teardown

## Symptom

`test_move_group_launch_smoke.py::test_launch_reaches_ready_state` failed on
amd64 with
`AssertionError: Proc drqp_joystick_translator-7 exited with code -2`. The
MoveIt stack reached ready, and the failure was only the clean-exit check after
teardown. The same commit passed on arm64, and the amd64 job passed on re-run.
That makes this an intermittent shutdown flake, not a regression from the PR,
which changed only CI config and a Renovate script.

## Reproduction

Seen once: CI run 38319897008, attempt 1, job
`CI/CD / Build and test ROS2 workspace (amd64)`, on #561 (head `089de46`,
2026-10-12). Attempt 2 passed. No local reproduction yet. Repeated runs of
`colcon test --packages-select drqp_moveit` would show the failure rate.

## Root cause

Not confirmed. What is known:

- An exit code of -2 means the process was terminated by signal 2. A Python
  process reports this when a `KeyboardInterrupt` escapes to the top level, so
  the interrupt was not caught by the `except` in `main()`.
- `main()` catches `KeyboardInterrupt` and `ExternalShutdownException` only
  around `rclpy.spin()`. A `KeyboardInterrupt` raised inside the `finally` block
  (`destroy_node()` / `rclpy.shutdown()`), or before the `try` is entered, is
  uncaught.
- `bringup.launch.py` has started the translator by default since
  `load_joystick_translator` was added with default `true`
  ([Bug: deployed robot has no joystick translator](deployed-robot-has-no-joystick-translator.md)).
  So every launch test that includes bringup now runs it.
- `assert_processes_exited_cleanly` ignores Python nodes that are known to die
  in rclpy teardown (`drqp_brain`, `drqp_robot_state`) through
  `DEFAULT_SHUTDOWN_KILLED_PROCESSES`. The translator is not on that list.

## Fix

Unfixed. The options, in order of preference:

- **Harden `main()`.** Catch `KeyboardInterrupt` and `ExternalShutdownException`
  in the teardown path as well, so a late SIGINT ends the process with code 0.
  To test it first, send SIGINT to the node during teardown and assert exit code
  0\.
- **Ignore it in the check.** Add `drqp_joystick_translator` to
  `DEFAULT_SHUTDOWN_KILLED_PROCESSES`. This is cheap, but it hides a real
  teardown defect.
- **Retry.** Add `@pytest.mark.flaky(retries=3)` to the smoke test, as the other
  `drqp_moveit` launch tests do. This is safe because the launch fixture is
  function-scoped.

## Key references

Verified anchor points (line numbers as of 2026-10-13):

- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:268` —
  `main()`
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:278` —
  `except (KeyboardInterrupt, ExternalShutdownException)` covers spin only
- `packages/runtime/drqp_brain/drqp_brain/joystick_translator_node.py:280` —
  `finally` teardown, not guarded
- `packages/runtime/drqp_moveit/test/test_move_group_launch_smoke.py:74` —
  `test_launch_reaches_ready_state`
- `packages/runtime/drqp_moveit/test/test_move_group_launch_smoke.py:80` —
  `assert_processes_exited_cleanly(proc_info)`
- `packages/runtime/drqp_launch_testing/drqp_launch_testing/process_exit_codes.py:62`
  — `DEFAULT_SHUTDOWN_KILLED_PROCESSES`
- `packages/runtime/drqp_brain/launch/bringup.launch.py:129` — translator
  `Node`, gated on `load_joystick_translator`
