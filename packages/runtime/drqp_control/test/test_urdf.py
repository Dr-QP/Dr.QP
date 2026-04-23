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

import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest


@pytest.fixture
def urdf_file():
    """Make URDF file path."""
    package_path = get_package_share_directory('drqp_control')
    urdf_file_path = os.path.join(package_path, 'urdf', 'dr_qp.urdf.xacro')
    return urdf_file_path


def test_urdf_file_exists(urdf_file):
    """Check if URDF file exists."""
    assert os.path.exists(urdf_file)


def test_urdf_file_is_valid(urdf_file):
    """Check if URDF file is valid."""
    with tempfile.NamedTemporaryFile(suffix='.urdf') as temp_file:
        try:
            proc = subprocess.run(
                ['xacro', urdf_file, '-o', temp_file.name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            assert proc.returncode == 0, 'xacro failed:\n' + proc.stdout.decode()
            assert os.path.exists(temp_file.name)

            proc = subprocess.run(
                ['check_urdf', temp_file.name],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
            )
            assert proc.returncode == 0, 'check_urdf failed:\n' + proc.stdout.decode()

            root = ET.parse(temp_file.name).getroot()
            links = {link.attrib['name'] for link in root.findall('link')}
            joints = {joint.attrib['name']: joint for joint in root.findall('joint')}

            assert 'dr_qp/imu_link' in links
            assert 'base_center_to_imu' in joints
            imu_joint = joints['base_center_to_imu']
            assert imu_joint.attrib['type'] == 'fixed'
            assert imu_joint.find('parent').attrib['link'] == 'dr_qp/base_center_link'
            assert imu_joint.find('child').attrib['link'] == 'dr_qp/imu_link'
        finally:
            if os.path.exists(temp_file.name):
                os.remove(temp_file.name)
