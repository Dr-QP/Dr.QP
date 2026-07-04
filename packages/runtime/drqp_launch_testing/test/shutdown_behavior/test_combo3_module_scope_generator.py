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
Combo 3: non-function-scoped (module-scoped) generator launch test.

Post-shutdown work from a non-function-scoped generator test used to be
unsupported: launch_pytest called ``FixtureManager.getfixtureinfo(funcargs=True)``
for the auto-generated shutdown item, a keyword the installed pytest no longer
accepts, raising ``TypeError``. The vendored ``launch_pytest``
(``packages/vendor/launch/launch_pytest``, see ``source-info.yaml``) drops that
stale ``funcargs`` kwarg, so this combo is now fully supported: the generator
yields once for the active phase and resumes for the shutdown phase, both
sharing the one module-scoped simulation.
"""

import launch_pytest
from probe_support import make_probe_launch, recorded_exit_codes
import pytest


@launch_pytest.fixture(scope='module')
def generate_test_description():
    return make_probe_launch()


@pytest.mark.launch(fixture=generate_test_description)
def test_generator_post_shutdown(generate_test_description):
    _ld, proc_info, _launch_id = generate_test_description
    yield
    assert recorded_exit_codes(proc_info)
