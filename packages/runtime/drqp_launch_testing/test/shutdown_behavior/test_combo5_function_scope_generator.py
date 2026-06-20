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
Combo 5: function-scoped generator test (yield once).

Result: a single launch test that yields exactly once runs its body before
shutdown and its post-yield code after the SAME simulation has been torn down,
so it observes that simulation's exit codes. This is the correct pattern for a
file with a single launch test (no separate shutdown function is needed).
"""

import launch_pytest
from probe_support import make_probe_launch, recorded_exit_codes
import pytest


@launch_pytest.fixture
def generate_test_description():
    return make_probe_launch()


@pytest.mark.launch(fixture=generate_test_description)
def test_active_then_exit_codes(generate_test_description):
    _ld, proc_info, _launch_id = generate_test_description
    # --- pre-shutdown phase: simulation is running here ---
    assert not recorded_exit_codes(proc_info), 'no process should have exited yet'
    yield
    # --- post-shutdown phase: same simulation has been torn down ---
    exits = recorded_exit_codes(proc_info)
    assert exits, 'generator post-yield must observe a populated proc_info'
    assert all(code == 0 for code in exits.values()), exits
