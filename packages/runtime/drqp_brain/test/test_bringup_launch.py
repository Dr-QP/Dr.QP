# Copyright (c) 2017-2025 Anton Matosov
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

from pathlib import Path


def test_bringup_launch_loads_the_imu_node():
    """Ensure bringup wires the Dr.QP IMU node into robot startup."""
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    launch_source = launch_file.read_text(encoding='utf-8')

    assert "name='load_imu'" in launch_source
    assert "executable='drqp_imu'" in launch_source
    assert 'UnlessCondition(use_gazebo)' in launch_source


def test_bringup_launch_uses_same_imu_transform_path_in_gazebo_and_hardware_issue356():
    """Issue 356: Gazebo should not bypass the base-to-IMU transform path."""
    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'
    launch_source = launch_file.read_text(encoding='utf-8')

    assert "'transform_imu_to_base_frame': IfElseSubstitution(" not in launch_source
