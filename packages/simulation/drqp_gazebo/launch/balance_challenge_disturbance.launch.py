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

from launch import LaunchDescription
from launch.actions import ExecuteProcess

DISTURBANCE_TOPIC = '/balance_challenge/motor_tilt_platform/target_position'
TILT_UP_POSITION = 0.2
TILT_DOWN_POSITION = 0.0
TILT_UP_HOLD_SECONDS = 5
TILT_DOWN_HOLD_SECONDS = 5
INITIAL_DELAY_SECONDS = 5


def generate_launch_description():
    disturbance_loop_script = (
        f'sleep {INITIAL_DELAY_SECONDS}; '
        'while true; do '
        f'gz topic -t {DISTURBANCE_TOPIC} -m gz.msgs.Double -p "data: {TILT_UP_POSITION}"; '
        f'sleep {TILT_UP_HOLD_SECONDS}; '
        f'gz topic -t {DISTURBANCE_TOPIC} -m gz.msgs.Double -p "data: {TILT_DOWN_POSITION}"; '
        f'sleep {TILT_DOWN_HOLD_SECONDS}; '
        'done'
    )

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=['bash', '-c', disturbance_loop_script],
                name='balance_challenge_disturbance_loop',
                output='screen',
            )
        ]
    )
