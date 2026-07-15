#!/usr/bin/env python3
#
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

import fcntl
import os
from pathlib import Path


class InstanceAlreadyRunningError(RuntimeError):
    """Raised when another instance already holds an instance guard lock."""


class InstanceGuard:
    """Hold an advisory file lock to prevent duplicate local process instances."""

    def __init__(self, name: str, lock_dir: Path | str | None = None):
        self.name = name
        self._lock_path = (Path(lock_dir) if lock_dir is not None else default_lock_dir()) / (
            f'{name}.lock'
        )
        self._lock_file = None

    @property
    def lock_path(self) -> Path:
        """Return the lock file path used by this guard."""
        return self._lock_path

    def acquire(self) -> None:
        """Acquire the instance lock or raise when another process owns it."""
        if not self.try_acquire():
            raise InstanceAlreadyRunningError(
                f'Another {self.name} process already holds the startup lock.'
            )

    def try_acquire(self) -> bool:
        """Try to acquire the instance lock without raising on contention."""
        if self._lock_file is not None:
            return True

        self._lock_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock_file = self._lock_path.open('w', encoding='utf-8')
        try:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self._lock_file.close()
            self._lock_file = None
            return False

        self._lock_file.seek(0)
        self._lock_file.truncate()
        self._lock_file.write(f'{os.getpid()}\n')
        self._lock_file.flush()
        return True

    def release(self) -> None:
        """Release the lock if it is currently held."""
        if self._lock_file is None:
            return
        try:
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            self._lock_file.close()
            self._lock_file = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.release()


def get_runtime_directory(app_name: str = 'drqp_brain') -> Path:
    """
    Return the writable runtime directory for this package.

    Prefer ROS_HOME so runtime files align with ROS conventions instead of the
    repository checkout location.
    """
    ros_home = os.environ.get('ROS_HOME')
    if ros_home:
        return Path(ros_home).expanduser() / app_name

    return Path.home() / '.ros' / app_name


def default_lock_dir() -> Path:
    """Return the ROS-local temporary directory for instance locks."""
    return get_runtime_directory() / 'tmp'


def make_launch_instance_guard(name: str):
    """Create a launch action that holds a domain-scoped lock for its lifetime."""
    from launch.actions import OpaqueFunction

    return OpaqueFunction(function=_acquire_launch_instance_guard, args=(name,))


def _domain_scoped_guard_name(name: str, ros_domain_id: str) -> str:
    """Return the instance guard name for a ROS domain."""
    return f'{name}-domain-{ros_domain_id}'


def _acquire_launch_instance_guard(context, name: str):
    from launch.actions import EmitEvent, LogError, RegisterEventHandler
    from launch.event_handlers import OnShutdown
    from launch.events import Shutdown

    guard = InstanceGuard(
        _domain_scoped_guard_name(name, context.environment.get('ROS_DOMAIN_ID', '0'))
    )
    if not guard.try_acquire():
        reason = f'{guard.name} is already running; refusing duplicate launch.'
        return [LogError(msg=reason), EmitEvent(event=Shutdown(reason=reason))]

    def release_guard(_event, _context):
        guard.release()

    return [RegisterEventHandler(OnShutdown(on_shutdown=release_guard))]
