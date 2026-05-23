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
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import os
import unittest

from moveit_launch_smoke_test_support import build_smoke_test_description
import pytest


@pytest.mark.launch_test
@pytest.mark.skipif(
    not os.environ.get('DISPLAY'),
    reason='RViz smoke test requires a graphical display',
)
def generate_test_description():
    return build_smoke_test_description(
        'demo.launch.py',
        launch_arguments={'show_rviz': 'true'},
        ready_delay=5.0,
    )


class TestDemoLaunchRvizSmoke(unittest.TestCase):
    """Smoke test for the demo.launch.py RViz visualization."""

    def test_only_one_rviz_process_starts(self, proc_info):
        rviz_processes = [name for name in proc_info.process_names() if 'rviz2' in name]
        self.assertEqual(
            len(rviz_processes),
            1,
            f'Expected exactly one RViz process, found: {rviz_processes}',
        )
