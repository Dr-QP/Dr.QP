# launch_pytest `shutdown=True` behavior matrix

This directory is an **executable specification** of how `launch_pytest`
shutdown handling interacts with fixture scope, for the `launch_pytest`
(3.4.x) + `pytest` (8.x) versions shipped with ROS 2 Jazzy in this workspace.

Each `test_combo*.py` is a runnable, asserting example of one row below. They
exist to (a) document the behavior our `drqp_launch_testing` helpers rely on and
(b) act as a regression alarm if a future toolchain changes that behavior.

## How "same vs separate simulation" is detected

`probe_support.make_probe_launch()` returns `(launch_description, proc_info,
launch_id)`. `launch_id` comes from a process-wide counter incremented on every
launch-description creation, i.e. once per launch fixture invocation. If an
active test and a shutdown test observe the **same** `launch_id`, they shared one
simulation; different ids mean separate simulations.

`proc_info` is a `ProcInfoHandler` populated by an `OnProcessExit` handler
(`track_process_exit_codes`). A shutdown body can only verify exit codes if it
runs **after** the simulation it cares about has shut down AND that simulation
actually started (hence the `ready_delay` before `ReadyToTest`).

## The matrix

| # | Fixture scope | Shutdown pattern                  | Shares the active sim? | Shutdown body sees exit codes? | Verdict                                |
| - | ------------- | --------------------------------- | ---------------------- | ------------------------------ | -------------------------------------- |
| 1 | function      | separate `shutdown=True` function | ❌ separate sim        | only of its own throwaway sim  | ✗ don't use for exit checks            |
| 2 | class         | separate `shutdown=True` function | ❌ separate sim        | only of its own throwaway sim  | ✗ don't use for exit checks            |
| 3 | class/module  | generator method (yield)          | n/a                    | n/a                            | ✗ **unsupported** (raises `TypeError`) |
| 4 | module        | separate `shutdown=True` function | ✅ same sim            | ✅ yes                         | ✓ use for multi-test files             |
| 5 | function      | generator test (yield once)       | ✅ same sim            | ✅ yes                         | ✓ use for single-test files            |
| 6 | any           | `@pytest.mark.flaky` retry        | n/a                    | n/a                            | ✓ crash-safe; only combo 5 relaunches  |

### Why combo 3 is unsupported

A non-function-scoped generator launch test makes `launch_pytest` build a
separate shutdown item via
`FixtureManager.getfixtureinfo(..., funcargs=True)`. The installed pytest removed
the `funcargs` keyword, so this raises `TypeError`. The combo 3 test is
`xfail(strict=True)`: if a future pairing fixes it, the test XPASSes and we
revisit the recommendations.

### Why combo 6 is "crash-safe" but not "relaunch-safe" for combos 1/2/4

`launch_pytest`'s `pytest_pyfunc_call` hookwrapper mutates `pyfuncitem.obj` in
place every time it runs, dispatching on whatever `pyfuncitem.obj` currently
is. Retried by `pytest-retry`, the second invocation sees the *previous
attempt's wrapper* (closed over an already-torn-down `event_loop`), and
re-wrapping it crashes with `RuntimeError: Event loop is closed` / `is already
running`. `drqp_launch_testing.launch_pytest_retry` fixes this by always
re-wrapping from a cached pristine original, never from `pyfuncitem.obj`.

Separately (and independent of that fix): `pytest-retry` tears down and
recreates function/class-scoped fixtures on retry, but does **not** re-run a
`module`-scoped fixture's setup on retry (verified with a bare, non-launch
module fixture — a `pytest-retry` limitation, not a `launch_pytest` one). So
after the patch:

- Combo 5 (function-scoped generator): retry genuinely relaunches. Useful for
  rescuing a real nondeterministic shutdown-crash flake.
- Combos 1, 2, 4 (module/class-scoped shared fixture): retry is crash-safe but
  reuses the same, already-recorded simulation — it re-checks identical data
  and can never rescue a real flake. `@pytest.mark.flaky` there is harmless but
  pointless; don't add it expecting it to help.

## Recommended patterns (consequences)

- **A file with a single launch test that needs post-shutdown checks** → combo 5:
  a function-scoped generator test that `yield`s once, then asserts on `proc_info`
  after the `yield`.
- **A file whose several launch tests must share one simulation** → combo 4:
  a `module`-scoped launch fixture, plain test functions, and one separate
  `shutdown=True` test that asserts on `proc_info`.
- **Never** rely on a separate `shutdown=True` function at function or class
  scope to verify the active test's processes (combos 1 & 2) — it runs against a
  different, throwaway simulation.

See `.github/instructions/launch-testing.instructions.md` for the project rules
derived from this matrix.
