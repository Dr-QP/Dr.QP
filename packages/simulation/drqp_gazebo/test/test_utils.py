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

"""Shared test utilities for Gazebo simulation tests."""

import inspect
import subprocess


def _caller_trace(max_depth: int = 3) -> str:
    """Return a compact caller chain for debugging test teardown/setup flows."""
    stack = inspect.stack()
    # Stack[0] is _caller_trace and stack[1] is the direct wrapper/caller in this module.
    callers = stack[2:][:max_depth]
    if not callers:
        return '<unknown caller>'

    return ' <- '.join(f'{frame.function}@{frame.filename}:{frame.lineno}' for frame in callers)


def ensure_gz_sim_not_running():
    """
    Remove any remaining Gazebo processes.

    This is needed to ensure clean test isolation.
    See https://github.com/ros2/launch/issues/545 for details.
    """
    trace = _caller_trace()
    print(
        f'Ensuring no gz sim processes are running... [pkill -9 -f "^gz sim"] caller={trace}',
        flush=True,
    )
    shell_cmd = ['pkill', '-9', '-f', '^gz sim']
    subprocess.run(shell_cmd, check=False)
