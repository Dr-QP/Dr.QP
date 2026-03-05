# # Copyright (c) 2017-2026 Anton Matosov
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

import unittest

from launch_testing import asserts
from launch_testing.proc_info_handler import ProcInfoHandler
from test_utils import ensure_gz_sim_not_running


class SimulationShutdownTestCase(unittest.TestCase):
    """Verify processes exit cleanly after the launch test finishes."""

    @classmethod
    def tearDownClass(cls) -> None:
        super().tearDownClass()
        ensure_gz_sim_not_running()

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally (except Gazebo)."""
        # Gazebo is SIGTERMed by the launch file, so we need to filter it out
        proc_info = self._filter_out_gazebo(proc_info)
        asserts.assertExitCodes(proc_info)

    def _filter_out_gazebo(self, proc_info):
        """Filter out Gazebo processes from the list."""
        filtered_proc_info = ProcInfoHandler()
        skipped_procs = ('gazebo', 'gz', 'bridge_node')
        for proc_name in proc_info.process_names():
            if not any(skip in proc_name for skip in skipped_procs):
                filtered_proc_info.append(proc_info[proc_name])
        return filtered_proc_info
