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

from drqp_brain.joystick_input_handler import ControlMode
from drqp_interfaces.msg import MovementCommandConstants
from drqp_keyboard_control.control_state import GuiControlState, VirtualAxes
import pytest


def test_wasd_key_down_up_holds_until_release():
    """Keyboard axes should use key state instead of repeat timeouts."""
    state = GuiControlState(sensitivity=0.5)

    assert state.key_down('w') is True
    assert state.axes().left_y == pytest.approx(0.5)
    assert state.axes().left_y == pytest.approx(0.5)

    assert state.key_up('w') is True
    assert state.axes().left_y == pytest.approx(0.0)


def test_repeated_key_down_reports_no_change():
    """OS key-repeat events should not report state changes."""
    state = GuiControlState()

    assert state.key_down('w') is True
    assert state.key_down('w') is False
    assert state.key_up('w') is True
    assert state.key_up('w') is False


def test_multiple_keyboard_keys_combine_across_sticks():
    """WASD and arrow keys should work while held together."""
    state = GuiControlState(sensitivity=0.75)

    for key in ('w', 'd', 'left', 'up'):
        state.key_down(key)

    axes = state.axes()
    assert axes.left_x == pytest.approx(0.75)
    assert axes.left_y == pytest.approx(0.75)
    assert axes.right_x == pytest.approx(-0.75)
    assert axes.right_y == pytest.approx(0.75)


def test_opposite_keyboard_keys_cancel_on_axis():
    """Opposite held keys should cancel each other."""
    state = GuiControlState(sensitivity=0.6)

    state.key_down('w')
    state.key_down('s')
    state.key_down('left')
    state.key_down('right')

    axes = state.axes()
    assert axes.left_y == pytest.approx(0.0)
    assert axes.right_x == pytest.approx(0.0)


def test_pointer_stick_takes_precedence_over_keyboard_until_released():
    """Dragged virtual sticks should override keyboard axes for that stick."""
    state = GuiControlState(sensitivity=0.5)
    state.key_down('w')

    state.set_left_stick(-0.25, -0.5)
    assert state.axes().left_x == pytest.approx(-0.25)
    assert state.axes().left_y == pytest.approx(-0.5)

    state.key_down('d')
    assert state.axes().left_x == pytest.approx(-0.25)

    state.release_left_stick()
    assert state.axes().left_x == pytest.approx(0.5)
    assert state.axes().left_y == pytest.approx(0.5)


def test_pointer_stick_clamps_to_unit_circle():
    """Pointer input should be limited to the unit circle."""
    state = GuiControlState()

    state.set_left_stick(3.0, 4.0)

    assert state.axes().left_x == pytest.approx(0.6)
    assert state.axes().left_y == pytest.approx(0.8)


def test_walk_mode_stick_release_springs_to_center():
    """In Walk mode releasing the stick must stop the robot."""
    state = GuiControlState()

    state.set_right_stick(0.7, -0.2)
    state.release_right_stick()

    assert state.axes().right_x == pytest.approx(0.0)
    assert state.axes().right_y == pytest.approx(0.0)


def test_body_mode_stick_release_latches_in_place():
    """In body modes releasing the stick must keep its value."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyPosition)

    state.set_left_stick(0.3, 0.6)
    state.release_left_stick()

    assert state.axes().left_x == pytest.approx(0.3)
    assert state.axes().left_y == pytest.approx(0.6)
    assert state.left_stick.latched is True

    command = state.movement_command()
    assert command.body_translation.x == pytest.approx(0.6)
    assert command.body_translation.y == pytest.approx(-0.3)


def test_body_rotation_stick_release_latches_in_place():
    """Body rotation mode should latch exactly like body position mode."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyRotation)

    state.set_right_stick(-0.4, 0.0)
    state.release_right_stick()

    command = state.movement_command()
    assert command.body_rotation.z == pytest.approx(0.4)


def test_body_pose_survives_switch_to_walk_mode():
    """Switching to Walk must keep the latched body pose in commands."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.2, 0.5)
    state.release_left_stick()

    state.set_control_mode(ControlMode.Walk)
    command = state.movement_command()

    assert command.body_translation.x == pytest.approx(0.5)
    assert command.body_translation.y == pytest.approx(-0.2)
    assert command.stride_direction.x == pytest.approx(0.0)
    assert command.stride_direction.y == pytest.approx(0.0)


def test_mode_switch_restores_latched_stick_positions():
    """Re-entering a body mode should restore its latched stick values."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.25, -0.75)
    state.release_left_stick()

    state.set_control_mode(ControlMode.BodyRotation)
    assert state.axes().left_x == pytest.approx(0.0)

    state.set_control_mode(ControlMode.BodyPosition)
    assert state.axes().left_x == pytest.approx(0.25)
    assert state.axes().left_y == pytest.approx(-0.75)


def test_leaving_walk_mode_stops_walking():
    """Stride and rotation must be zeroed when leaving Walk mode."""
    state = GuiControlState(sensitivity=1.0)
    state.key_down('w')
    state.movement_command()

    state.set_control_mode(ControlMode.BodyPosition)
    command = state.movement_command()

    assert command.stride_direction.x == pytest.approx(0.0)
    assert command.stride_direction.y == pytest.approx(0.0)
    assert command.rotation_speed == pytest.approx(0.0)


def test_entering_walk_mode_resumes_keyboard_and_springs_pointer_input():
    """Walk mode entry should reflect held keys, not stale latched values."""
    state = GuiControlState(sensitivity=0.5)
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.9, 0.0)
    state.release_left_stick()
    state.key_down('w')

    state.set_control_mode(ControlMode.Walk)

    assert state.axes().left_x == pytest.approx(0.0)
    assert state.axes().left_y == pytest.approx(0.5)


def test_keyboard_press_in_body_mode_overrides_latch():
    """Fresh keyboard input should take over a latched stick."""
    state = GuiControlState(sensitivity=0.5)
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.9, 0.9)
    state.release_left_stick()

    state.key_down('s')

    assert state.axes().left_x == pytest.approx(0.0)
    assert state.axes().left_y == pytest.approx(-0.5)


def test_keyboard_release_in_body_mode_latches_value():
    """Releasing the last motion key in a body mode should hold the value."""
    state = GuiControlState(sensitivity=0.4)
    state.set_control_mode(ControlMode.BodyRotation)

    state.key_down('a')
    state.key_up('a')

    assert state.axes().left_x == pytest.approx(-0.4)
    assert state.left_stick.latched is True


def test_sensitivity_adjustment_applies_to_held_keys():
    """Changing sensitivity while keys are held should rescale the stick."""
    state = GuiControlState(sensitivity=0.5, sensitivity_step=0.1)
    state.key_down('w')

    state.key_down('+')

    assert state.sensitivity == pytest.approx(0.6)
    assert state.axes().left_y == pytest.approx(0.6)


def test_sensitivity_adjustment_is_clamped():
    """Plus and minus keys should adjust the emulated stick magnitude."""
    state = GuiControlState(sensitivity=0.95, sensitivity_step=0.1)

    state.key_down('+')
    assert state.sensitivity == pytest.approx(1.0)

    for _ in range(20):
        state.key_down('-')

    assert state.sensitivity == pytest.approx(0.1)


def test_sensitivity_adjustment_keeps_latched_sticks():
    """Sensitivity changes must not clobber a latched pointer value."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.3, 0.3)
    state.release_left_stick()

    state.adjust_sensitivity(0.1)

    assert state.axes().left_x == pytest.approx(0.3)
    assert state.axes().left_y == pytest.approx(0.3)


def test_tab_cycles_control_modes():
    """TAB should cycle through the same control modes as the joystick node."""
    state = GuiControlState()

    state.key_down('tab')
    assert state.control_mode == ControlMode.BodyPosition

    state.key_down('tab')
    assert state.control_mode == ControlMode.BodyRotation

    state.key_down('tab')
    assert state.control_mode == ControlMode.Walk


def test_control_mode_is_read_only_attribute():
    """Direct assignment must be rejected; set_control_mode owns transitions."""
    state = GuiControlState()

    with pytest.raises(AttributeError):
        state.control_mode = ControlMode.BodyPosition


def test_number_keys_select_gaits():
    """Number keys should directly select tripod, ripple, and wave gaits."""
    state = GuiControlState()

    state.key_down('2')
    assert state.gait == MovementCommandConstants.GAIT_RIPPLE

    state.key_down('3')
    assert state.gait == MovementCommandConstants.GAIT_WAVE

    state.key_down('1')
    assert state.gait == MovementCommandConstants.GAIT_TRIPOD


def test_trigger_values_clamp():
    """Trigger setters should clamp into the 0..1 range."""
    state = GuiControlState()

    state.set_left_trigger(1.4)
    state.set_right_trigger(-0.2)

    assert state.left_trigger == pytest.approx(1.0)
    assert state.right_trigger == pytest.approx(0.0)


def test_walk_command_maps_sticks_and_left_trigger():
    """Walk mode should map left stick, right stick X, and left trigger."""
    state = GuiControlState()
    state.set_left_stick(0.25, 0.5)
    state.set_right_stick(-0.75, 0.1)
    state.set_left_trigger(0.4)

    command = state.movement_command()

    assert command.stride_direction.x == pytest.approx(0.5)
    assert command.stride_direction.y == pytest.approx(-0.25)
    assert command.stride_direction.z == pytest.approx(0.4)
    assert command.rotation_speed == pytest.approx(0.75)
    assert command.gait_type == MovementCommandConstants.GAIT_TRIPOD


def test_body_position_mode_uses_sticks():
    """Body position mode should match existing joystick-style mapping."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(-0.4, 0.3)
    state.set_right_stick(0.0, 0.8)

    command = state.movement_command()

    assert command.body_translation.x == pytest.approx(0.3)
    assert command.body_translation.y == pytest.approx(0.4)
    assert command.body_translation.z == pytest.approx(0.8)
    assert command.stride_direction.x == pytest.approx(0.0)


def test_body_rotation_mode_uses_sticks():
    """Body rotation mode should match existing joystick-style mapping."""
    state = GuiControlState()
    state.set_control_mode(ControlMode.BodyRotation)
    state.set_left_stick(0.6, -0.5)
    state.set_right_stick(0.7, 0.0)

    command = state.movement_command()

    assert command.body_rotation.x == pytest.approx(-0.6)
    assert command.body_rotation.y == pytest.approx(-0.5)
    assert command.body_rotation.z == pytest.approx(-0.7)
    assert command.rotation_speed == pytest.approx(0.0)


def test_reset_motion_inputs_clears_everything():
    """Reset should zero sticks, triggers, body pose, and mode memory."""
    state = GuiControlState()
    state.key_down('w')
    state.set_left_trigger(0.7)
    state.set_control_mode(ControlMode.BodyPosition)
    state.set_left_stick(0.5, 0.5)
    state.release_left_stick()
    state.movement_command()

    state.reset_motion_inputs()
    command = state.movement_command()

    assert state.axes() == VirtualAxes()
    assert command.body_translation.x == pytest.approx(0.0)
    assert command.stride_direction.x == pytest.approx(0.0)

    state.set_control_mode(ControlMode.BodyRotation)
    state.set_control_mode(ControlMode.BodyPosition)
    assert state.axes().left_x == pytest.approx(0.0)
