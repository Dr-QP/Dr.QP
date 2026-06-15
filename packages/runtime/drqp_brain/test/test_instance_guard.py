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

from drqp_brain.instance_guard import (
    default_lock_dir,
    get_runtime_directory,
    InstanceAlreadyRunningError,
    InstanceGuard,
)
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


def test_instance_guard_allows_startup_after_release():
    """Release the singleton lock when the owning process exits."""
    lock_dir = Path('.tmp')

    with InstanceGuard('test_drqp_brain_released', lock_dir=lock_dir):
        pass

    with InstanceGuard('test_drqp_brain_released', lock_dir=lock_dir):
        pass
