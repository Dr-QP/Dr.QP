# Copyright (c) 2017-2026 Anton Matosov
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

"""Unit tests for the process exit-code verification helpers."""

from drqp_launch_testing import assert_processes_exited_cleanly
from launch_testing.proc_info_handler import ProcInfoHandler
import pytest


class _FakeAction:
    """Minimal stand-in for a launch ExecuteProcess action."""

    def __init__(self, name: str) -> None:
        self.process_details = {'name': name}


class _FakeProcessExited:
    """Minimal stand-in for a launch ProcessExited event."""

    def __init__(self, name: str, returncode: int) -> None:
        self.action = _FakeAction(name)
        self.process_name = name
        self.returncode = returncode


def _proc_info(*items: tuple[str, int]) -> ProcInfoHandler:
    proc_info = ProcInfoHandler()
    for name, returncode in items:
        proc_info.append(_FakeProcessExited(name, returncode))
    return proc_info


def test_all_clean_exits_pass():
    assert_processes_exited_cleanly(_proc_info(('drqp_brain', 0), ('robot_state_publisher', 0)))


def test_nonzero_unlisted_process_raises():
    with pytest.raises(AssertionError):
        assert_processes_exited_cleanly(_proc_info(('drqp_brain', 5)))


def test_ignored_process_nonzero_is_tolerated():
    # gazebo is SIGTERM'd on shutdown (-15) but allowlisted; drqp_brain must be clean.
    assert_processes_exited_cleanly(_proc_info(('gazebo-2', -15), ('drqp_brain', 0)))


def test_only_ignored_processes_pass():
    assert_processes_exited_cleanly(_proc_info(('gazebo', -15), ('move_group', -15)))


def test_empty_proc_info_passes():
    assert_processes_exited_cleanly(_proc_info())


def test_custom_ignore_list():
    assert_processes_exited_cleanly(_proc_info(('flaky_thing', 3)), ignore=('flaky_thing',))
    with pytest.raises(AssertionError):
        assert_processes_exited_cleanly(_proc_info(('flaky_thing', 3)), ignore=('other',))
