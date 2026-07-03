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
Combo 6: ``@pytest.mark.flaky`` retry on a combo-4 (module-scope) shutdown=True test.

Without ``drqp_launch_testing.launch_pytest_retry`` active (this directory's
``conftest.py`` registers it), retrying a launch_pytest test crashes with
"RuntimeError: Event loop is closed" / "is already running" instead of
retrying, because launch_pytest re-wraps the previous attempt's stale wrapper.
See ``launch_pytest_retry.py`` for the root cause and fix.

This test also documents a second, independent limitation: ``pytest-retry``'s
retry mechanism forces teardown of function/class-scoped fixtures, but does
**not** actually re-run a `module`-scoped fixture's setup on retry (verified
with a bare, non-launch_pytest module fixture -- this is a pytest-retry
limitation, not a launch_pytest one). Practically: for a combo-4 shared
simulation, ``@pytest.mark.flaky`` is crash-safe after this patch, but a retry
re-checks the *same, already-recorded* ``proc_info`` rather than relaunching --
it cannot rescue a genuine nondeterministic shutdown-crash flake for this
shape. Combo 5 (function-scoped generator) does not have this limitation: its
fixture is torn down and recreated on every test regardless of retry, so a
retry there genuinely relaunches. Prefer combo 5 when retry needs to be useful,
not just safe.
"""

import launch_pytest
from probe_support import make_probe_launch, recorded_exit_codes
import pytest

_SEEN: dict = {'attempts': []}


@launch_pytest.fixture(scope='module')
def generate_test_description():
    return make_probe_launch()


@pytest.mark.launch(fixture=generate_test_description)
def test_active(generate_test_description):
    _ld, _proc_info, launch_id = generate_test_description
    _SEEN['active'] = launch_id


@pytest.mark.flaky(retries=2)
@pytest.mark.launch(fixture=generate_test_description, shutdown=True)
def test_retry_does_not_crash(generate_test_description):
    _ld, proc_info, launch_id = generate_test_description
    _SEEN['attempts'].append(launch_id)
    assert recorded_exit_codes(proc_info), 'shutdown body must observe a populated proc_info'
    # Fail on the first attempt only, forcing pytest-retry to retry.
    assert len(_SEEN['attempts']) > 1, f"intentional failure on attempt {len(_SEEN['attempts'])}"


def test_retry_reuses_same_simulation():
    """Document (not just tolerate) that combo 4 + retry does not relaunch."""
    attempts = _SEEN['attempts']
    assert len(attempts) == 2, f'expected exactly one retry, got {attempts}'
    assert attempts[0] == attempts[1], (
        'if this starts failing, pytest-retry now recreates module-scoped fixtures on '
        'retry -- revisit the combo 4 + flaky recommendation in SPEC.md/instructions'
    )
