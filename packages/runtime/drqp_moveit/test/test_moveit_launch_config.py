# Copyright (c) 2026 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Tests for MoveIt launch argument ownership and smoke-test overrides."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.utilities import perform_substitutions
from moveit_launch_smoke_test_support import (
    build_smoke_test_description,
    READY_TIMEOUT,
)


def _load_move_group_launch_module():
    launch_path = Path(__file__).parents[1] / 'launch' / 'move_group.launch.py'
    spec = importlib.util.spec_from_file_location('move_group_launch', launch_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_move_group_launch_owns_initial_state_timeout() -> None:
    """Keep the production timeout configurable with its existing default."""
    module = _load_move_group_launch_module()
    captured_parameters = {}

    def capture_node(**kwargs):
        """Capture the final nested override instead of starting move_group."""
        captured_parameters.update(kwargs['parameters'][-1])
        return LogInfo(msg='Captured move_group node parameters')

    module.Node = capture_node
    launch_description = module.generate_launch_description()
    timeout_argument = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == 'wait_for_initial_state_timeout'
    )

    assert perform_substitutions(LaunchContext(), timeout_argument.default_value) == '10.0'

    context = LaunchContext()
    context.launch_configurations['wait_for_initial_state_timeout'] = str(READY_TIMEOUT)
    timeout_parameter = captured_parameters['planning_scene_monitor_options'][
        'wait_for_initial_state_timeout'
    ]
    assert timeout_parameter.evaluate(context) == READY_TIMEOUT


def test_smoke_description_matches_move_group_timeout_to_watchdog() -> None:
    """Allow move_group to wait as long as the smoke readiness watchdog."""
    launch_description = build_smoke_test_description(
        'demo.launch.py',
        launch_arguments={'show_rviz': 'false'},
    )
    include = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    )

    launch_arguments = dict(include.launch_arguments)
    assert launch_arguments['show_rviz'] == 'false'
    assert launch_arguments['wait_for_initial_state_timeout'] == str(READY_TIMEOUT)
