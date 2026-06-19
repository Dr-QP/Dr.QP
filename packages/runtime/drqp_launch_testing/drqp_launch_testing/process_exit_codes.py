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

"""
Per-process exit-code verification for launch_pytest tests.

launch_pytest's ``shutdown=True`` tests only assert the aggregate launch
*service* return code, which is unaffected by managed processes exiting
non-zero. To recover the per-process exit-code checking that the older
launch_testing post-shutdown tests provided, register an exit-code recorder on
the launch description with :func:`track_process_exit_codes` and assert clean
exits from a ``shutdown=True`` test with :func:`assert_processes_exited_cleanly`.

Typical usage in a functions-only launch_pytest module::

    @launch_pytest.fixture(scope='module')
    def generate_test_description():
        launch_description = build_my_launch_description()
        proc_info = track_process_exit_codes(launch_description)
        return launch_description, proc_info

    @pytest.mark.launch(fixture=generate_test_description, shutdown=True)
    def test_processes_exit_cleanly(generate_test_description):
        _launch_description, proc_info = generate_test_description
        assert_processes_exited_cleanly(proc_info)
"""

from collections.abc import Iterable

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_testing import asserts
from launch_testing.proc_info_handler import ProcInfoHandler

# Processes that the Dr.QP simulation/MoveIt launch files tear down with SIGTERM
# on shutdown; they routinely report a non-zero return code on exit, so they are
# excluded from the clean-exit assertion by default. Membership is by substring
# match against the process name. The list is validated by the slow Gazebo/MoveIt
# CI suites that actually exercise process teardown.
DEFAULT_SHUTDOWN_KILLED_PROCESSES = ('gazebo', 'gz', 'bridge_node', 'move_group')


def track_process_exit_codes(launch_description: LaunchDescription) -> ProcInfoHandler:
    """
    Record every process exit on ``launch_description`` for later verification.

    Adds an ``OnProcessExit`` handler that appends each ``ProcessExited`` event to
    the returned :class:`ProcInfoHandler`. Return it alongside the launch
    description from a launch fixture so a ``shutdown=True`` test can assert exit
    codes once the launch service has fully shut down.
    """
    proc_info = ProcInfoHandler()

    def _record(event, _context):
        proc_info.append(event)

    launch_description.add_action(RegisterEventHandler(OnProcessExit(on_exit=_record)))
    return proc_info


def assert_processes_exited_cleanly(
    proc_info: ProcInfoHandler,
    *,
    ignore: Iterable[str] = DEFAULT_SHUTDOWN_KILLED_PROCESSES,
) -> None:
    """
    Assert every recorded process exited cleanly, ignoring known SIGTERM victims.

    Processes whose name contains any token in ``ignore`` are excluded (e.g. the
    simulator and MoveIt processes the launch file kills with SIGTERM). All
    remaining processes must have exited with code 0.
    """
    filtered = ProcInfoHandler()
    for name in proc_info.process_names():
        if not any(token in name for token in ignore):
            filtered.append(proc_info[name])
    if not filtered.process_names():
        # Nothing to check (e.g. the launch produced only ignored processes); the
        # launch-service return code assertion and the test bodies cover that.
        return
    asserts.assertExitCodes(filtered)
