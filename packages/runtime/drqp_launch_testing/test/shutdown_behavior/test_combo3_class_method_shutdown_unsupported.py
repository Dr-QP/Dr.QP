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
Combo 3: non-function-scoped generator launch test (post-shutdown on a method).

Attempting to do post-shutdown work from a non-function-scoped generator test
(the natural way to add a shutdown phase to a class/module test) is UNSUPPORTED
with the installed launch_pytest + pytest combination: launch_pytest calls
``FixtureManager.getfixtureinfo(funcargs=True)``, a keyword the installed pytest
no longer accepts, raising ``TypeError`` for the auto-generated shutdown item.

This module is xfail(strict=True): if a future launch_pytest/pytest pairing fixes
this, the test will XPASS and prompt us to revisit the recommended patterns.
"""

import launch_pytest
from probe_support import make_probe_launch, recorded_exit_codes
import pytest


@launch_pytest.fixture(scope='module')
def generate_test_description():
    return make_probe_launch()


@pytest.mark.xfail(
    reason='launch_pytest calls getfixtureinfo(funcargs=True), unsupported by installed pytest',
    strict=True,
    raises=TypeError,
)
@pytest.mark.launch(fixture=generate_test_description)
def test_generator_post_shutdown(generate_test_description):
    _ld, proc_info, _launch_id = generate_test_description
    yield
    assert recorded_exit_codes(proc_info)
