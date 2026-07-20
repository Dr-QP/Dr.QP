# Copyright (c) 2026 Anton Matosov
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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Lock down the bounded-execution policy for slow Gazebo CI."""

from pathlib import Path

import conftest
import pytest
import test_robot_smoke

WORKSPACE_ROOT = Path(__file__).resolve().parents[4]


class _CollectionItem:
    """Minimal pytest item used to exercise the collection policy."""

    def __init__(self, *keywords: str) -> None:
        self.keywords = dict.fromkeys(keywords)
        self.markers: list[pytest.MarkDecorator] = []

    def add_marker(self, marker: pytest.MarkDecorator) -> None:
        """Record a marker added by the collection hook."""
        self.markers.append(marker)


def test_ordinary_launch_item_is_not_retried(monkeypatch: pytest.MonkeyPatch) -> None:
    """Behavior failures must run once unless a test opts into retry."""
    monkeypatch.setenv('DRQP_TEST_MODE', 'slow')
    item = _CollectionItem('launch')

    conftest.pytest_collection_modifyitems(None, [item])

    assert all(marker.name != 'flaky' for marker in item.markers)


def test_shutdown_crash_smoke_test_keeps_explicit_retry_marker() -> None:
    """The evidenced simulator shutdown crash remains locally retried."""
    markers = getattr(test_robot_smoke.test_robot_smoke, 'pytestmark', ())
    flaky_markers = [marker for marker in markers if marker.name == 'flaky']

    assert len(flaky_markers) == 1
    assert flaky_markers[0].kwargs == {'retries': 3}


def test_slow_matrix_runs_ctest_serially() -> None:
    """The slow devcontainer matrix must not use automatic CTest parallelism."""
    workflow = (WORKSPACE_ROOT / '.github/workflows/ci.yml').read_text()

    assert "sim_test_mode: 'slow'\n            ctest_parallel_level: '1'" in workflow
    assert 'CTEST_PARALLEL_LEVEL=${{ matrix.ctest_parallel_level }}' in workflow


def test_every_gazebo_launch_executable_has_finite_outer_timeout() -> None:
    """Ensure CTest stops a stuck executable before the 60-minute job limit."""
    cmake = (WORKSPACE_ROOT / 'packages/simulation/drqp_gazebo/test/CMakeLists.txt').read_text()

    assert 'set(drqp_gazebo_launch_test_timeout 1800)' in cmake
    assert 'TIMEOUT ${drqp_gazebo_launch_test_timeout}' in cmake
    assert 'TIMEOUT 0' not in cmake


def test_test_artifacts_are_published_even_after_failure_or_cancellation() -> None:
    """Preserve xUnit and captured logs whenever the runner leaves them behind."""
    workflow = (WORKSPACE_ROOT / '.github/workflows/ci.yml').read_text()
    publish_steps = workflow.count(
        'if: ${{ always() }}\n        uses: ./.github/actions/publish-test-results'
    )

    assert publish_steps == 2
