---
name: ros2-launch-management
description: Create, compose, configure, and debug ROS 2 Python launch files in this workspace. Use when adding a launch file, launch arguments, node actions, includes, parameter files, or startup troubleshooting; use launch-testing to test launched processes and ros2-lifecycle-management for lifecycle transitions.
---

# ROS 2 Launch Management

Use Python launch files to start processes and pass their configuration. Keep
the file responsible for composition, not for application behavior or a
distributed deployment design.

## Run launch commands reliably

Run every ROS command from the workspace root through the wrapper:

```bash
scripts/with-ros-env.sh ros2 launch <package_name> <launch_file>.launch.py
```

In Codex, request sandbox escalation when launching or inspecting a running
ROS graph, because those commands need ROS sockets, processes, and log writes.
If the wrapper cannot source ROS on the host, use the escalation path in
[ros2-workspace-build](../ros2-workspace-build/SKILL.md); do not retry the
same command locally. Build an edited package incrementally before launch as
described there.

Use [launch-testing](../launch-testing/SKILL.md) for assertions about launched
processes, including clean exit codes. Use
[ros2-lifecycle-management](../ros2-lifecycle-management/SKILL.md) to manage
lifecycle state transitions. This skill does not design multi-machine
networking, fault-recovery state machines, or node internals.

## Create a launch file

Place Python launch files in `<package>/launch/` and name them
`<purpose>.launch.py`. Declare only arguments that this file owns. For a
single node, keep the action and its parameters close together:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<node_executable>',
            name='<node_name>',
            output='screen',
            parameters=[{'update_rate': 50.0}],
        ),
    ])
```

Use a parameter YAML file when values vary by deployment. Resolve an installed
file through the package share directory; do not rely on the source-tree
working directory:

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = Path(get_package_share_directory('<package_name>')) / 'config' / 'params.yaml'
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='<node_executable>',
            parameters=[str(params_file)],
        ),
    ])
```

Install the files. For `ament_cmake`, install both directories:

```cmake
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
```

For `ament_python`, include launch files in `setup.py` (and import `glob` and
`os`):

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
```

Declare the ROS packages used at runtime in `package.xml`, for example
`launch`, `launch_ros`, and `ament_index_python` when applicable.

## Add owned arguments and conditions

Use a declared argument and `LaunchConfiguration` for a choice owned by this
file. `UnlessCondition` is imported from `launch.conditions`.

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = DeclareLaunchArgument('use_sim', default_value='false')
    hardware_node = Node(
        package='drqp_serial',
        executable='serial_node',
        condition=UnlessCondition(LaunchConfiguration('use_sim')),
    )
    return LaunchDescription([use_sim, hardware_node])
```

Invoke it with `use_sim:=true`; discover arguments with:

```bash
scripts/with-ros-env.sh ros2 launch <package_name> <launch_file>.launch.py --show-args
```

## Compose launch files without stealing their arguments

Use `IncludeLaunchDescription` and an installed package-share path to compose
files. Do **not** redeclare, bind, or explicitly forward arguments owned by
the included file: values supplied to the outer launch are already available
through the launch context. Add `launch_arguments={...}.items()` only for a
value owned and intentionally mapped by the current file.

```python
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    included_file = Path(get_package_share_directory('drqp_serial')) / 'launch' / 'driver.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(included_file))),
    ])
```

Use event handlers only for local launch-process ordering or cleanup. For a
lifecycle startup sequence, route to the lifecycle skill rather than encoding
state transitions ad hoc in a general launch file.

## Debug and validate

First inspect the installed artifacts and request the launch system's debug
output. Put the `--debug` option before the package positional arguments.

```bash
scripts/with-ros-env.sh ros2 pkg prefix <package_name>
scripts/with-ros-env.sh ros2 launch --debug <package_name> <launch_file>.launch.py
scripts/with-ros-env.sh ros2 run <package_name> <node_executable>
```

A bare `python3 <launch_file>` only defines functions; it does not validate
the description. As a quick construction check, import the file and call its
factory (replace the placeholder path):

```bash
scripts/with-ros-env.sh python3 -c "import importlib.util; p='<launch_file_path>'; s=importlib.util.spec_from_file_location('under_test', p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); assert m.generate_launch_description() is not None"
```

Then use [launch-testing](../launch-testing/SKILL.md) when behavior, process
startup, shutdown, parameters, or exit codes need verification. For a running
system, inspect names, parameters, and logs with
[ros2-diagnostics](../ros2-diagnostics/SKILL.md).

## Related skills

- [ros2-workspace-build](../ros2-workspace-build/SKILL.md)
- [launch-testing](../launch-testing/SKILL.md)
- [ros2-lifecycle-management](../ros2-lifecycle-management/SKILL.md)
- [ros2-diagnostics](../ros2-diagnostics/SKILL.md)
