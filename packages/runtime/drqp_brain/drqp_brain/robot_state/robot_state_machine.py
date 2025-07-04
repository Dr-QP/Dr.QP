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
    finalized = State(final=True)

    # Transitions/Events
    initialize = torque_off.to(initializing)
    turn_on = (
        initializing.to(torque_on)
        | torque_off.to(
            torque_on
        )  # allow to turn on from torque off state until init sequence is built
    )
    turn_off = torque_on.to(torque_off) | initializing.to(torque_off) | finalizing.to(torque_off)
    finalize = torque_on.to(finalizing)
    done = finalizing.to(finalized)

    kill_switch_on = turn_off
    kill_switch_off = turn_on
