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

from drqp_brain.instance_guard import InstanceAlreadyRunningError, InstanceGuard
import pytest


def test_instance_guard_uses_tmp_lock_file():
    """Store locks under the repository-local temporary directory by default."""
    guard = InstanceGuard('test_drqp_brain')

    assert guard.lock_path == Path('.tmp') / 'test_drqp_brain.lock'


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
