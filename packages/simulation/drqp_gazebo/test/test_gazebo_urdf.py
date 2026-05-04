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

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory


def test_gazebo_urdf_converts_to_sdf_without_warnings():
    """Check that the Gazebo-enabled Dr.QP URDF converts to SDF cleanly."""
    package_path = get_package_share_directory('drqp_control')
    urdf_file = os.path.join(package_path, 'urdf', 'drqp.urdf.xacro')

    with tempfile.NamedTemporaryFile(suffix='.urdf') as temp_file:
        try:
            proc = subprocess.run(
                ['xacro', urdf_file, 'use_gazebo:=true', '-o', temp_file.name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            assert proc.returncode == 0, 'xacro failed:\n' + proc.stdout.decode()

            proc = subprocess.run(
                ['gz', 'sdf', '-p', temp_file.name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            sdf_output = proc.stdout.decode()

            assert proc.returncode == 0, 'gz sdf failed:\n' + sdf_output
            assert '<sdf' in sdf_output, 'Expected gz sdf to print converted SDF output'
            assert 'Warning' not in sdf_output, 'gz sdf emitted warnings:\n' + sdf_output
        finally:
            if os.path.exists(temp_file.name):
                os.remove(temp_file.name)