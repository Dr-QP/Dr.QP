# Copyright (c) 2017-2025 Anton Matosov
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

"""Which nodes bringup starts for a given set of launch arguments."""

from collections import Counter
import importlib.util
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'


def _load_launch_description():
    spec = importlib.util.spec_from_file_location('bringup_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _children(entity):
    if isinstance(entity, LaunchDescription):
        return entity.entities
    return entity.get_sub_entities()


def _walk(entities):
    for entity in entities:
        yield entity
        yield from _walk(_children(entity))


def _started_executables(overrides: dict[str, str]) -> set[str]:
    """Evaluate launch conditions and return the executables that would run."""
    description = _load_launch_description()
    context = LaunchContext()
    for entity in _walk(description.entities):
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value:
            default = perform_substitutions(context, entity.default_value)
            context.launch_configurations.setdefault(entity.name, default)
    context.launch_configurations.update(overrides)

    def visit(entities):
        for entity in entities:
            condition = getattr(entity, 'condition', None)
            if condition is not None and not condition.evaluate(context):
                continue
            if isinstance(entity, Node):
                yield entity.node_executable
            yield from visit(_children(entity))

    return set(visit(description.entities))


def test_default_bringup_starts_joystick_translator():
    """The deployed robot runs bringup with no arguments and needs the translator."""
    started = _started_executables({})

    assert 'drqp_joystick_translator' in started
    assert 'game_controller_node' not in started


def test_load_joystick_starts_game_controller_alongside_translator():
    started = _started_executables({'load_joystick': 'true'})

    assert 'game_controller_node' in started
    assert 'drqp_joystick_translator' in started


def test_translator_can_be_disabled():
    started = _started_executables({'load_joystick_translator': 'false'})

    assert 'drqp_joystick_translator' not in started


@pytest.mark.parametrize('name', ['load_joystick', 'load_joystick_translator'])
def test_joystick_arguments_are_declared_once(name):
    declared = Counter(
        entity.name
        for entity in _walk(_load_launch_description().entities)
        if isinstance(entity, DeclareLaunchArgument)
    )

    assert declared[name] == 1
