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

"""Shared probe launch description for the shutdown-behavior examples."""

from drqp_launch_testing import track_process_exit_codes
import launch
from launch.actions import ExecuteProcess, TimerAction
from launch_pytest.actions import ReadyToTest

# Monotonic counter incremented on every launch description creation. Each
# launch_pytest fixture invocation produces a new simulation, so comparing the
# ``launch_id`` seen by an active test against the one seen by a shutdown test
# reveals whether they share a single simulation.
_LAUNCH_COUNT = {'value': 0}


def make_probe_launch(ready_delay: float = 0.5):
    """
    Return ``(launch_description, proc_info, launch_id)`` for a probe process.

    The probe process handles SIGINT/SIGTERM and exits 0, so a clean shutdown is
    recorded in ``proc_info``. ``ready_delay`` gives the process time to start
    before tests run, which is required for the shutdown body to observe a
    populated ``proc_info``.
    """
    _LAUNCH_COUNT['value'] += 1
    launch_id = _LAUNCH_COUNT['value']
    launch_description = launch.LaunchDescription(
        [
            ExecuteProcess(
                cmd=['bash', '-c', 'trap "exit 0" INT TERM; while true; do sleep 0.1; done'],
                name='probe_proc',
            ),
            TimerAction(period=ready_delay, actions=[ReadyToTest()]),
        ]
    )
    proc_info = track_process_exit_codes(launch_description)
    return launch_description, proc_info, launch_id


def recorded_exit_codes(proc_info) -> dict:
    """Return a ``{process_name: returncode}`` snapshot of ``proc_info``."""
    return {name: proc_info[name].returncode for name in proc_info.process_names()}
