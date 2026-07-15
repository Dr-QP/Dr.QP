---
name: ros2-parameter-tuning
description: Configure, validate, install, inspect, and safely tune ROS 2 node parameters in YAML or at runtime. Use when creating parameter files, validating parameter behavior, changing a running node’s settings, or diagnosing parameter loading. Use ros2-launch-management for launch ownership and ros2-diagnostics for graph-level failures.
---

# ROS 2 Parameter Tuning

Keep static configuration, non-mutating validation, and runtime tuning
separate. A valid YAML file does not prove that a node declares, accepts, or
uses each parameter.

## Run Commands Reliably

Run ROS commands through the workspace overlay wrapper:

```bash
scripts/with-ros-env.sh ros2 <verb> <arguments>
```

In Codex, request sandbox escalation for ROS discovery sockets, node processes,
or ROS log writes and state that reason in the request. If ROS is unavailable
locally, use the workspace container/Codespaces escalation path instead of
retrying unsourced commands. Use `./.tmp/` for temporary dumps and snapshots.

## Create and Install Static Configuration

Use the node’s actual fully-qualified name, or `/**` only when the file is
intentionally namespace-independent:

```yaml
/**:
  ros__parameters:
    control_frequency: 50.0
    max_velocity: 1.5
    enable_safety: true
    joint_names: [joint1, joint2]
```

Install `config/` with the package type that owns it. For `ament_cmake`:

```cmake
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```

For `ament_python`, include the files in `setup.py`:

```python
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/<package_name>']),
    ('share/<package_name>', ['package.xml']),
    ('share/<package_name>/config', glob('config/*.yaml')),
]
```

Build only the package and what it needs after changing installed files:

```bash
scripts/with-ros-env.sh colcon build --packages-up-to <package_name> \
  --symlink-install
```

Load a package-installed file from a launch file owned by that package. Do not
re-declare or forward launch arguments that belong to an included launch file.

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

params_file = (
    Path(get_package_share_directory('<package_name>'))
    / 'config'
    / 'params.yaml'
)
node = Node(
    package='<package_name>',
    executable='<node_name>',
    parameters=[str(params_file)],
)
```

Use `ros2-launch-management` when launch ownership or substitutions are in
question.

## Validate Without Changing a Running Node

Use `rclpy`'s ROS parameter-file parser to validate both YAML and the ROS
parameter-file shape without contacting or changing a running node:

```bash
scripts/with-ros-env.sh python3 - config/params.yaml <<'PY'
import sys

import yaml
from rclpy.parameter import parameter_dict_from_yaml_file

parameter_file = sys.argv[1]
try:
    parameters = parameter_dict_from_yaml_file(
        parameter_file, use_wildcard=True
    )
except (OSError, RuntimeError, TypeError, ValueError, yaml.YAMLError) as error:
    raise SystemExit(
        f'Invalid ROS parameter file {parameter_file}: {error}'
    ) from error

if not parameters:
    raise SystemExit(
        f'Invalid ROS parameter file {parameter_file}: no parameters found'
    )
PY
```

The command exits nonzero with `Invalid ROS parameter file ...` for unreadable
files, YAML/parser errors, invalid ROS node mappings, or a file with no
parameters. It validates syntax and ROS parameter-file structure only; it does
not confirm that a particular node declares or accepts the values.

Then inspect what a running node declares and currently uses. These commands do
not change it:

```bash
scripts/with-ros-env.sh ros2 param list /<node_name>
scripts/with-ros-env.sh ros2 param describe /<node_name> <parameter_name>
scripts/with-ros-env.sh ros2 param get /<node_name> <parameter_name>
mkdir -p ./.tmp
scripts/with-ros-env.sh ros2 param dump /<node_name> > ./.tmp/parameters-before.yaml
```

Jazzy’s `ros2 param dump` writes YAML to standard output; it has no
`--output-dir` option. Treat `ros2 param load` as a runtime mutation, not a
syntax check.

## Tune a Running Node Safely

Capture the original value, make one change, and verify both the accepted value
and observable behavior. Do not batch unrelated tuning changes.

```bash
scripts/with-ros-env.sh ros2 param get /<node_name> max_velocity
scripts/with-ros-env.sh ros2 param set /<node_name> max_velocity 1.2
scripts/with-ros-env.sh ros2 param get /<node_name> max_velocity
scripts/with-ros-env.sh ros2 topic echo /parameter_events --once --timeout 5
```

If the node rejects the request or behavior regresses, restore the captured
value and verify it. For a multi-parameter rollback, snapshot first and load
only the specific prior file with authorization:

```bash
scripts/with-ros-env.sh ros2 param set /<node_name> max_velocity <previous_value>
scripts/with-ros-env.sh ros2 param get /<node_name> max_velocity
scripts/with-ros-env.sh ros2 param load /<node_name> ./.tmp/parameters-before.yaml
```

`param get` confirms the node’s stored value, not that its controller or cache
uses the value. Pair it with the relevant safety, output, or diagnostics check.

## Implement Dynamic Parameters Correctly

Declare parameters and validate proposed changes before committing them to
cached state. Rejecting a bad update keeps both the ROS parameter value and the
cache unchanged.

Python (`rclpy`) example:

```python
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class Controller(Node):
    def __init__(self) -> None:
        super().__init__('controller')
        self.declare_parameter('max_velocity', 1.0)
        self.max_velocity = self.get_parameter('max_velocity').value
        self.add_on_set_parameters_callback(self._validate_and_apply)

    def _validate_and_apply(
        self, parameters: list[Parameter]
    ) -> SetParametersResult:
        candidate = self.max_velocity
        for parameter in parameters:
            if parameter.name == 'max_velocity':
                is_valid = (
                    parameter.type_ == Parameter.Type.DOUBLE
                    and 0.0 < parameter.value <= 3.0
                )
                if not is_valid:
                    return SetParametersResult(
                        successful=False, reason='max_velocity must be in (0, 3]'
                    )
                candidate = parameter.value
        self.max_velocity = candidate
        return SetParametersResult(successful=True)
```

For a C++ node, use `add_on_set_parameters_callback`, validate the complete
candidate set, and update the cached member only after validation succeeds.
Keep a callback handle as a member so the callback remains registered. Test
both an accepted update and a rejected update; the latter must leave the cached
value unchanged.

## Boundaries and Handoffs

Use this skill to configure or tune parameter values. Use
`ros2-diagnostics` to establish whether a graph/QoS/runtime problem is actually
parameter-related, `ros2-launch-management` to change launch behavior, and
`ros2-workspace-testing` to run focused tests after code changes.

## References

- [ROS 2 parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [Using ros2 param](https://docs.ros.org/en/jazzy/How-To-Guides/Using-ros2-param.html)
- [Parameters in a C++ class](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
