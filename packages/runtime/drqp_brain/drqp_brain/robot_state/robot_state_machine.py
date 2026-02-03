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


from statemachine import State, StateMachine


class RobotStateMachine(StateMachine):
    """A robot state machine."""

    # States
    torque_off = State(initial=True)
    initializing = State()
    torque_on = State()
    finalizing = State()
    finalized = State()
    servos_rebooting = State()

    # Transitions/Events
    initialize = torque_off.to(initializing) | finalized.to(initializing)
    initializing_done = initializing.to(torque_on)
    turn_off = (
        torque_on.to(torque_off)
        | initializing.to(torque_off)
        | finalizing.to(torque_off)
        | finalized.to(torque_off)
    )
    finalize = torque_on.to(finalizing)
    finalizing_done = finalizing.to(finalized)
    reboot_servos = (
        torque_off.to(servos_rebooting)
        | initializing.to(servos_rebooting)
        | torque_on.to(servos_rebooting)
        | finalizing.to(servos_rebooting)
        | finalized.to(servos_rebooting)
    )
    servos_rebooting_done = servos_rebooting.to(torque_off)

    kill_switch_pressed = turn_off | initialize
