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
Combo 4: module-scoped fixture + separate module-level ``shutdown=True`` test.

Result: the active test(s) and the shutdown test share ONE simulation, and the
shutdown body observes the fully-populated exit codes. This is the correct
pattern when several test functions must share a single simulation (e.g.
test_robot_control_nodes_and_clock).
"""

import launch_pytest
from probe_support import make_probe_launch, recorded_exit_codes
import pytest

_SEEN = {}


@launch_pytest.fixture(scope='module')
def generate_test_description():
    return make_probe_launch()


@pytest.mark.launch(fixture=generate_test_description)
def test_active_one(generate_test_description):
    _ld, _proc_info, launch_id = generate_test_description
    _SEEN['active'] = launch_id


@pytest.mark.launch(fixture=generate_test_description)
def test_active_two(generate_test_description):
    _ld, _proc_info, launch_id = generate_test_description
    # Same simulation is reused across module-scoped test functions.
    assert launch_id == _SEEN['active']


@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_shutdown(generate_test_description):
    _ld, proc_info, launch_id = generate_test_description
    assert launch_id == _SEEN['active'], 'module scope must share one simulation'
    exits = recorded_exit_codes(proc_info)
    assert exits, 'shutdown body must observe a populated proc_info'
    assert all(code == 0 for code in exits.values()), exits
