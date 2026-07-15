---
name: add-test-file-python
description: Add or modify focused pytest unit, rclpy node-integration, or launch_pytest test files for an existing Python ROS 2 package. Use for Python TDD, pytest fixtures, ROS node tests, and package test-dependency integration; route launch-test behavior rules to launch-testing.
---

# Add Python Test File

Add a focused pytest test for an existing Python source module. For C++ tests,
use [add-test-file-cpp](../add-test-file-cpp/).

## Classify the test before writing it

Read the source and nearby tests, then choose the narrowest useful type:

| Need                                                 | Test type        | Placement                           |
| ---------------------------------------------------- | ---------------- | ----------------------------------- |
| Pure function or class behavior                      | Unit             | `test/test_<module>.py`             |
| A node's callbacks, timers, or ROS graph interaction | Node integration | `test/test_<module>_integration.py` |
| Multiple processes or a launch description           | Launch           | `test/test_<feature>_launch.py`     |

Keep one behavior family per file. When a target file already exists, extend it
only when its fixture lifetime and purpose remain the same; otherwise add a
separate, clearly named file.

## Write pytest-native tests

Use pytest functions and fixtures, never `unittest`. Test observable behavior,
including invalid inputs and error paths, rather than implementation details.
For an rclpy fixture, initialize ROS only if necessary, yield the node or test
harness, destroy nodes in teardown, and call `rclpy.try_shutdown()` only when
the fixture owns that initialization. Keep subscriptions, publishers, timers,
and clients alive for the duration of the assertion.

Add the package's standard test dependencies when absent:
`ament_copyright`, `ament_flake8`, `ament_pep257`, and `python3-pytest`.
For `ament_python`, retain the package's pytest discovery bridge in
`test/__init__.py`; do not modify `setup.py` merely to add a test file.

## Route launch tests to the dedicated policy

For launch tests, use [launch-testing](../launch-testing/) as the single source
of truth for fixture scope, `ReadyToTest`, generator tests, shutdown behavior,
retry, and per-process exit-code assertions. Add both dependencies if absent:

```xml
<test_depend>launch_pytest</test_depend>
<test_depend>drqp_launch_testing</test_depend>
```

The latter supplies the required per-process exit-code helper; a bare
`shutdown=True` test is insufficient. The supporting
[executable behavior matrix](../../../packages/runtime/drqp_launch_testing/test/shutdown_behavior/SPEC.md)
explains why the fixture/scope choices matter.

## Run the narrow verification

Format and lint the changed Python test using
[python-format-lint](../python-format-lint/) when appropriate, then run the
package test through the ROS wrapper:

```bash
scripts/with-ros-env.sh python3 -m colcon test \
  --packages-select <package_name> --mixin coverage-pytest \
  --return-code-on-test-failure
```

Read `log/latest_test/<package_name>/stdout_stderr.log` after a failure. Use
[ros2-workspace-testing](../ros2-workspace-testing/) for test-result analysis,
failed-test reruns, and escalation when ROS is unavailable locally.

## Related resources

- [find-test-files](../find-test-files/)
- [launch-testing](../launch-testing/)
- [ros2-workspace-testing](../ros2-workspace-testing/)
