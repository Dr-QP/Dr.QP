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

from pathlib import Path

import drqp_brain.instance_guard as instance_guard
from drqp_brain.instance_guard import (
    _acquire_launch_instance_guard,
    default_lock_dir,
    domain_instance_guard,
    domain_scoped_guard_name,
    get_runtime_directory,
    InstanceAlreadyRunningError,
    InstanceGuard,
)
from launch.actions import EmitEvent, LogError
import pytest


def _workspace_root():
    return Path(__file__).resolve().parents[4]


def test_instance_guard_uses_ros_home_tmp_lock_file(monkeypatch):
    """Store locks under the ROS_HOME app runtime directory by default."""
    ros_home = Path.cwd() / '.tmp' / 'ros_home'
    monkeypatch.setenv('ROS_HOME', str(ros_home))

    guard = InstanceGuard('test_drqp_brain')

    assert get_runtime_directory() == ros_home / 'drqp_brain'
    assert default_lock_dir() == ros_home / 'drqp_brain' / 'tmp'
    assert guard.lock_path == ros_home / 'drqp_brain' / 'tmp' / 'test_drqp_brain.lock'


def test_runtime_directory_falls_back_to_home_ros(monkeypatch):
    """Use the ROS default home when ROS_HOME is not set."""
    monkeypatch.delenv('ROS_HOME', raising=False)
    monkeypatch.setenv('HOME', str(Path.cwd() / '.tmp' / 'home'))

    assert get_runtime_directory() == Path.home() / '.ros' / 'drqp_brain'


def test_instance_guard_uses_ros_home_tmp_from_package_subdirectory(monkeypatch):
    """Resolve the same lock path even when ROS launches from a package cwd."""
    workspace_root = _workspace_root()
    ros_home = workspace_root / '.tmp' / 'ros_home'
    monkeypatch.setenv('ROS_HOME', str(ros_home))
    monkeypatch.chdir(workspace_root / 'packages' / 'runtime' / 'drqp_brain')

    guard = InstanceGuard('test_drqp_brain')

    assert guard.lock_path == ros_home / 'drqp_brain' / 'tmp' / 'test_drqp_brain.lock'


def test_instance_guard_rejects_duplicate_from_different_cwd(monkeypatch):
    """Use one ROS_HOME lock file across different process working directories."""
    workspace_root = _workspace_root()
    ros_home = workspace_root / '.tmp' / 'ros_home'
    monkeypatch.setenv('ROS_HOME', str(ros_home))

    with InstanceGuard('test_drqp_brain_cwd'):
        monkeypatch.chdir(workspace_root / 'packages' / 'runtime' / 'drqp_brain')
        duplicate_guard = InstanceGuard('test_drqp_brain_cwd')

        with pytest.raises(InstanceAlreadyRunningError):
            duplicate_guard.acquire()


def test_instance_guard_rejects_duplicate_processes():
    """Refuse startup when another process holds the instance lock."""
    lock_dir = Path('.tmp')

    with InstanceGuard('test_drqp_brain', lock_dir=lock_dir):
        duplicate_guard = InstanceGuard('test_drqp_brain', lock_dir=lock_dir)

        with pytest.raises(InstanceAlreadyRunningError):
            duplicate_guard.acquire()


def test_instance_guard_try_acquire_returns_false_for_duplicate_processes():
    """Report lock contention without raising or waiting."""
    lock_dir = Path('.tmp')

    with InstanceGuard('test_drqp_brain_try_acquire', lock_dir=lock_dir):
        duplicate_guard = InstanceGuard('test_drqp_brain_try_acquire', lock_dir=lock_dir)

        assert not duplicate_guard.try_acquire()


def test_instance_guard_allows_startup_after_release():
    """Release the singleton lock when the owning process exits."""
    lock_dir = Path('.tmp')

    with InstanceGuard('test_drqp_brain_released', lock_dir=lock_dir):
        pass

    with InstanceGuard('test_drqp_brain_released', lock_dir=lock_dir):
        pass


@pytest.mark.parametrize(
    ('ros_domain_id', 'expected_name'),
    [
        ('0', 'drqp_gazebo_sim-domain-0'),
        ('42', 'drqp_gazebo_sim-domain-42'),
    ],
)
def test_launch_instance_guard_name_is_scoped_by_ros_domain(ros_domain_id, expected_name):
    """Give each ROS domain an independent launch-instance guard."""
    assert domain_scoped_guard_name('drqp_gazebo_sim', ros_domain_id) == expected_name


def test_domain_scoped_instance_guards_allow_parallel_domains():
    """Allow independent launch-instance guards in separate ROS domains."""
    lock_dir = Path('.tmp')

    with InstanceGuard(domain_scoped_guard_name('test_stack', '1'), lock_dir=lock_dir):
        with InstanceGuard(domain_scoped_guard_name('test_stack', '2'), lock_dir=lock_dir):
            pass


def test_domain_instance_guard_uses_current_ros_domain(monkeypatch):
    """Give process entrypoints independent locks in separate ROS domains."""
    lock_dir = Path('.tmp')
    monkeypatch.setenv('ROS_DOMAIN_ID', '17')

    with domain_instance_guard('test_runtime_node', lock_dir=lock_dir):
        duplicate_guard = domain_instance_guard('test_runtime_node', lock_dir=lock_dir)
        with pytest.raises(InstanceAlreadyRunningError):
            duplicate_guard.acquire()

        monkeypatch.setenv('ROS_DOMAIN_ID', '18')
        with domain_instance_guard('test_runtime_node', lock_dir=lock_dir):
            pass


def test_launch_instance_guard_shuts_down_when_lock_is_held(monkeypatch):
    """Stop the launch cleanly instead of raising when the guard is held."""

    class ContendedGuard:
        name = 'drqp_gazebo_sim-domain-42'

        def try_acquire(self):
            return False

    guard = ContendedGuard()
    monkeypatch.setattr(instance_guard, 'InstanceGuard', lambda _name: guard)

    context = type('LaunchContext', (), {'environment': {'ROS_DOMAIN_ID': '42'}})()
    actions = _acquire_launch_instance_guard(context, 'drqp_gazebo_sim')

    assert isinstance(actions[0], LogError)
    assert isinstance(actions[1], EmitEvent)
    assert actions[1].event.reason == (
        'drqp_gazebo_sim-domain-42 is already running; refusing duplicate launch.'
    )
