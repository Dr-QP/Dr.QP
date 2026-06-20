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
Combo 1: function-scoped fixture + separate ``shutdown=True`` test.

Result: the active test and the shutdown test each get their OWN simulation, so
the shutdown test CANNOT verify the active test's processes. Do not use this
pattern for exit-code checks; use combo 5 (a generator test) instead.
"""

import launch_pytest
from probe_support import make_probe_launch
import pytest

_SEEN = {}


@launch_pytest.fixture
def generate_test_description():
    return make_probe_launch()


@pytest.mark.launch(fixture=generate_test_description)
def test_active(generate_test_description):
    _ld, _proc_info, launch_id = generate_test_description
    _SEEN['active'] = launch_id


@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_shutdown(generate_test_description):
    _ld, _proc_info, launch_id = generate_test_description
    # Function scope re-invokes the fixture, so the shutdown test launches a
    # fresh, separate simulation rather than reusing the active test's one.
    assert launch_id != _SEEN['active'], (
        'function-scoped shutdown test unexpectedly shared the active sim'
    )
