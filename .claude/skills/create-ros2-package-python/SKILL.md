---
name: create-ros2-package-python
description: Create a ROS 2 Python (ament_python) package with its inner Python module, package metadata, console executable, and pytest scaffolding. Use when creating a new rclpy package, setting up setup.py or setup.cfg, or scaffolding a Python ROS 2 node in this workspace.
---

# Create ROS 2 Python Package

Create a focused `ament_python` package that follows the workspace layout and
can be built and tested immediately. For C++ packages, use
[create-ros2-package-cpp](../create-ros2-package-cpp/).

## Gather and validate inputs

Require a lowercase underscore-separated package name with the project
`drqp_` prefix, its purpose, and its runtime dependencies. Default the location
to `packages/runtime`; stop if the target directory already exists. Decide
whether the package needs a console node and whether it will contain launch
tests before creating files.

## Create the package layout

Keep importable code inside the inner package directory. A console executable
such as `drqp_example = drqp_example.example_node:main` must target a module
under `<package_root>/<package_name>/`, never a peer of `setup.py`.

```
<package_name>/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── resource/<package_name>
├── <package_name>/
│   ├── __init__.py
│   └── <node_name>_node.py
├── launch/
├── test/
│   ├── __init__.py
│   └── test_<package_name>.py
└── .coveragerc
```

Use `find_packages(exclude=['test', 'test.*'])` in `setup.py`; install the
resource marker and `package.xml` as data files. Add only the console-script
entries that package modules actually provide. Follow an existing package such
as `packages/runtime/drqp_brain/` for the local metadata shape.

## Implement each rclpy executable completely

For every entry point, provide a `main()` that initializes ROS, creates and
spins the node, then destroys it and shuts down in `finally`. Handle normal
shutdown without hiding unrelated startup or runtime errors:

```python
import rclpy
from rclpy.executors import ExternalShutdownException


def main() -> int:
    node = None
    try:
        rclpy.init()
        node = ExampleNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
```

Use `if __name__ == '__main__': raise SystemExit(main())` only for direct
module execution; the console script calls `main()` itself.

## Declare dependencies and tests

Use package format 3 with `ament_python` as the build type. Declare every ROS
and message runtime dependency used by the package, including `rclpy` for a
node. Add the workspace's standard test dependencies:
`ament_copyright`, `ament_flake8`, `ament_pep257`, and `python3-pytest`.
Declare `extras_require={'test': ['pytest']}` rather than obsolete
`tests_require`.

For a launch test, also add `<test_depend>launch_pytest</test_depend>` and
`<test_depend>drqp_launch_testing</test_depend>`; add `launch` and
`launch_ros` as runtime dependencies when the package ships or imports launch
files. Follow [launch-testing](../launch-testing/) for the required fixture and
exit-code pattern. For unit and node tests, use
[add-test-file-python](../add-test-file-python/).

## Verify with targeted ROS commands

Use the ROS wrapper, an incremental build, and a package-specific test run:

```bash
scripts/with-ros-env.sh python3 -m colcon build \
  --packages-up-to <package_name> --symlink-install
scripts/with-ros-env.sh python3 -m colcon test \
  --packages-select <package_name> --mixin coverage-pytest \
  --return-code-on-test-failure
```

Inspect `log/latest_build/` or
`log/latest_test/<package_name>/stdout_stderr.log` when either command fails.
Use [ros2-workspace-build](../ros2-workspace-build/) or
[ros2-workspace-testing](../ros2-workspace-testing/) for their full execution
and no-local-ROS escalation procedures.

## Related resources

- [ROS 2 package tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [create-ros2-package-cpp](../create-ros2-package-cpp/)
- [Coding conventions](../../../AGENTS.md)
