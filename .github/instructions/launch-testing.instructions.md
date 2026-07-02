---
description: 'Conventions for launch_pytest integration tests (process startup + per-process exit-code verification)'
applyTo: '**/test/**/*.py'
---

# Launch Test Conventions (launch_pytest)

For Python style see [python.instructions.md](./python.instructions.md). These
rules apply to `launch_pytest` tests that start ROS 2 processes via a
`LaunchDescription`. They are derived from an executable behavior matrix:
`packages/runtime/drqp_launch_testing/test/shutdown_behavior/` (see its
`SPEC.md`).

## Write launch tests as functions, not classes

Prefer plain module-level test functions over `unittest`-style test classes.
Share state (a node, a harness) through a `@pytest.fixture`, not `self`. Keep
`rclpy.init()` / `rclpy.try_shutdown()` in that fixture; do not add an explicit
`node.destroy_node()` — `rclpy.try_shutdown()` releases nodes.

## Verify per-process exit codes — never ship a no-op shutdown test

`launch_pytest`'s `shutdown=True` mechanism only asserts the aggregate launch
**service** return code; it does NOT check that managed processes exited 0. Use
the `drqp_launch_testing` helpers to recover per-process checking:

```python
from drqp_launch_testing import assert_processes_exited_cleanly, track_process_exit_codes

@launch_pytest.fixture(...)
def generate_test_description():
    launch_description = build_my_launch_description()
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info
```

Then assert with `assert_processes_exited_cleanly(proc_info)`. Simulator/MoveIt
processes that are SIGTERM'd on shutdown (`gazebo`, `gz`, `bridge_node`,
`move_group`) are allowlisted by default; pass `ignore=(...)` to customize.
Add `<test_depend>drqp_launch_testing</test_depend>` to the package's
`package.xml`.

## Choose the scope + shutdown pattern by test count

The two correct patterns (and the traps to avoid) come straight from the matrix:

- **Single launch test in the file → function scope (default) + a generator
  test.** The test `yield`s once: body before, exit-code check after, on the
  SAME simulation. No separate shutdown function.

  ```python
  @launch_pytest.fixture                      # function scope (default)
  def generate_test_description():
      ld = build_my_launch_description()
      return ld, track_process_exit_codes(ld)

  @pytest.mark.launch(fixture=generate_test_description)
  def test_node_behaviour(consumer, generate_test_description):
      consumer.do_checks()
      yield                                   # simulation tears down here
      assert_processes_exited_cleanly(generate_test_description[1])
  ```

- **Several launch tests must share ONE simulation → `scope='module'` +
  separate `shutdown=True` test.** Only use module scope when sharing is
  required (e.g. an expensive Gazebo launch reused by several test functions).

  ```python
  @launch_pytest.fixture(scope='module')
  def generate_test_description():
      ld = build_my_launch_description()
      return ld, track_process_exit_codes(ld)

  @pytest.mark.launch(fixture=generate_test_description)
  def test_one(robot): robot.assert_a()

  @pytest.mark.launch(fixture=generate_test_description)
  def test_two(robot): robot.assert_b()

  @pytest.mark.launch(fixture=generate_test_description, shutdown=True)
  def test_processes_exit_cleanly(generate_test_description):
      assert_processes_exited_cleanly(generate_test_description[1])
  ```

### Do not

- **Do not** pair a separate `shutdown=True` function with a **function**- or
  **class**-scoped fixture to check exit codes — it launches a _different,
  throwaway_ simulation, so it verifies nothing about the active test (matrix
  combos 1 & 2).
- **Do not** make a non-function-scoped (class/module) test a generator — the
  installed launch_pytest/pytest pairing raises `TypeError`
  (`getfixtureinfo(funcargs=True)`); matrix combo 3.
- **Do not** default to `scope='module'`/`'class'`. Function scope is the
  default; reach for module scope only to share one simulation across several
  tests.

## Retry known shutdown-crash flakiness — never mask a real bug

Gazebo/MoveIt launch tests occasionally crash during process shutdown for
reasons unrelated to the test body (a process race on teardown), which fails
the `assert_processes_exited_cleanly` check even though the test itself
passed. When CI history (repeated reruns of the same test, same failure
signature at the exit-code assertion) confirms this pattern for a specific
test, mark it `@pytest.mark.flaky(retries=3)` (from `pytest-retry`) stacked
above `@pytest.mark.launch(...)`:

```python
@pytest.mark.flaky(retries=3)
@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_processes_exit_cleanly(generate_test_description):
    assert_processes_exited_cleanly(generate_test_description[1])
```

- Add `<test_depend>pytest-retry</test_depend>` to the package's `package.xml`.
- `flaky` must be a registered marker in the root `pyproject.toml`
  `[tool.pytest.ini_options]` `markers` list.
- Retrying re-runs the whole test (the launch included), so a genuine
  assertion failure earlier in the test body reproduces identically on every
  retry and still fails the build — retries only rescue the nondeterministic
  shutdown-crash case, they never mask a real regression.
- Do **not** reach for `flaky` as a substitute for fixing the underlying
  shutdown crash; use it only for tests with a confirmed history of this
  specific failure mode, not preemptively.

## Fixture ordering gotcha

An `autouse`/shared fixture that assumes the launched system is running MUST
declare `generate_test_description` as a parameter (value unused, `# noqa:
ARG002`). `@pytest.mark.launch` is only an inherited marker; without the
parameter pytest may run setup before any process launches and discovery times
out. A `ready_delay` (`TimerAction` before `ReadyToTest`) is also required so
processes are up before tests — and so the shutdown body observes populated exit
codes.
