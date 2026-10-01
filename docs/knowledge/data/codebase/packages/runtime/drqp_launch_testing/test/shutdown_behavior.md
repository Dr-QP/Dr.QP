---
type: codebase
description: Executable specification of launch_pytest shutdown=True behavior across fixture scopes and pytest-retry, guarding the patterns the workspace's launch tests rely on.
source: packages/runtime/drqp_launch_testing/test/shutdown_behavior
source_digest: sha256:aa15f6c407f482e202728f94f0f0c5e53fc048d3e04b1330e31db00f52e40f58
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_launch_testing/test/shutdown_behavior
---

# shutdown_behavior

"An executable specification of how `launch_pytest` shutdown handling interacts
with fixture scope." It covers `launch_pytest` 3.4.x with pytest 8.x on Jazzy.
Each `test_combo*.py` asserts one row of the matrix in `SPEC.md`, so a toolchain
upgrade that changes the behavior fails here first.

## Public surface

- `SPEC.md`: the matrix and the recommended patterns.
  - **Single test that needs post-shutdown checks:** use combo 5, a
    function-scoped generator test that yields once.
  - **Several tests sharing one simulation:** use combo 4, a module-scoped
    fixture plus a separate `shutdown=True` test.
- `probe_support.make_probe_launch(ready_delay)` returns
  `(launch_description, proc_info, launch_id)`. `recorded_exit_codes(proc_info)`
  reads the results.

## How it works

`launch_id` comes from a process-wide counter that increments once per launch
fixture invocation. When an active test and a shutdown test see the same id,
they shared one launch.

| Combo | Scope and pattern                          | Result                                             |
| ----- | ------------------------------------------ | -------------------------------------------------- |
| 1     | function scope, separate shutdown function | separate launch; exit codes unusable               |
| 2     | class scope, separate shutdown function    | separate launch; exit codes unusable               |
| 3     | module scope, generator                    | shares the launch                                  |
| 4     | module scope, separate shutdown function   | shares the launch                                  |
| 5     | function scope, generator                  | shares the launch                                  |
| 6     | `@pytest.mark.flaky(retries=2)` on combo 4 | crash-safe; only combo 5 truly relaunches on retry |

## Depends on

- [drqp_launch_testing](../../drqp_launch_testing.md)
  (`track_process_exit_codes`)
- The vendored `launch_pytest` in `packages/vendor/launch/launch_pytest`

## Invariants & gotchas

- Combo 3 works only because the vendored `launch_pytest` drops a stale
  `funcargs=True` keyword. Stock `launch_pytest` raises `TypeError` there.
- Combo 6 depends on the vendored fix that re-wraps the test from the cached
  original callable. `pytest-retry` does not re-run module-scoped fixture setup,
  so retries on combos 1, 2, and 4 re-check the same recorded launch.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/SPEC.md` — matrix
  and recommendations
- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/probe_support.py:35`
  — `make_probe_launch`
- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/test_combo4_module_scope_separate_shutdown.py:36`
  — module-scoped fixture
- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/test_combo6_retry_compat.py:69`
  — `flaky(retries=2)`
