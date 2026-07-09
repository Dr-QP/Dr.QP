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

"""
Input model for the GUI keyboard controller.

Pygame-free and rclpy-free so the whole model is unit testable. Keyboard keys
and pointer drags drive two ``VirtualStick`` instances; ``GuiControlState``
turns them into ``MovementCommand`` messages, mirroring the semantics of the
physical joystick translator (``drqp_brain.joystick_translator_node``).

Latching: a real gamepad stick can be held while pressing the mode button, so
body pose survives mode switches. A mouse cannot hold a stick and click a
button at once, so in the body position/rotation modes the virtual sticks
latch in place on release instead of springing back to center. Walk mode
always springs back — releasing input must stop the robot.
"""

from dataclasses import dataclass

from drqp_brain.joystick_input_handler import all_control_modes, ControlMode
from drqp_interfaces.msg import MovementCommand, MovementCommandConstants
from geometry_msgs.msg import Vector3

from drqp_keyboard_control.layout import clamp, clamp_vector

LEFT_STICK_KEYS = frozenset({'w', 'a', 's', 'd'})
RIGHT_STICK_KEYS = frozenset({'up', 'down', 'left', 'right'})
MOTION_KEYS = LEFT_STICK_KEYS | RIGHT_STICK_KEYS

# Per-key unit contribution to the owning stick, scaled by sensitivity.
LEFT_KEY_VECTORS = {'d': (1.0, 0.0), 'a': (-1.0, 0.0), 'w': (0.0, 1.0), 's': (0.0, -1.0)}
RIGHT_KEY_VECTORS = {
    'right': (1.0, 0.0),
    'left': (-1.0, 0.0),
    'up': (0.0, 1.0),
    'down': (0.0, -1.0),
}

# Modes where stick values must survive pointer release (see module docstring).
LATCHING_MODES = frozenset({ControlMode.BodyPosition, ControlMode.BodyRotation})

ZERO_VECTOR = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class VirtualAxes:
    """Virtual controller axes used to build movement commands."""

    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0


@dataclass
class VirtualStick:
    """
    One virtual thumb stick driven by pointer drags or keyboard keys.

    ``dragging`` marks an active pointer drag (pointer wins over keyboard).
    ``latched`` marks a value held in place after release in a latching mode.
    """

    x: float = 0.0
    y: float = 0.0
    dragging: bool = False
    latched: bool = False

    @property
    def value(self) -> tuple[float, float]:
        """Return current (x, y) axes."""
        return (self.x, self.y)

    def set_axes(self, x: float, y: float):
        """Assign raw axis values."""
        self.x = x
        self.y = y

    def reset(self):
        """Return to center and clear pointer/latch flags."""
        self.x = 0.0
        self.y = 0.0
        self.dragging = False
        self.latched = False


class GuiControlState:
    """Track GUI and keyboard input as joystick-like movement state."""

    gaits = [
        MovementCommandConstants.GAIT_TRIPOD,
        MovementCommandConstants.GAIT_RIPPLE,
        MovementCommandConstants.GAIT_WAVE,
    ]

    def __init__(
        self,
        *,
        sensitivity: float = 0.5,
        sensitivity_step: float = 0.1,
        minimum_sensitivity: float = 0.1,
        maximum_sensitivity: float = 1.0,
    ):
        self.sensitivity = sensitivity
        self.sensitivity_step = sensitivity_step
        self.minimum_sensitivity = minimum_sensitivity
        self.maximum_sensitivity = maximum_sensitivity

        self.held_keys: set[str] = set()
        self.gait_index = 0
        self.left_stick = VirtualStick()
        self.right_stick = VirtualStick()
        self.left_trigger = 0.0
        self.right_trigger = 0.0

        # Persistent command state, mirroring JoystickInputHandler: only the
        # active mode's fields are driven by the axes, the rest keep their
        # last values so body pose survives mode switches.
        self.stride_direction = ZERO_VECTOR
        self.rotation_speed = 0.0
        self.body_translation = ZERO_VECTOR
        self.body_rotation = ZERO_VECTOR

        self._control_mode = ControlMode.Walk
        self._mode_stick_memory: dict[ControlMode, tuple] = {}

    @property
    def control_mode(self) -> ControlMode:
        """Return the current semantic control mode (set via set_control_mode)."""
        return self._control_mode

    # ---------------------------------------------------------------- keyboard

    def key_down(self, key: str) -> bool:
        """Apply a normalized key press and return whether state changed."""
        key = key.lower()
        if key in MOTION_KEYS:
            if key in self.held_keys:
                return False
            self.held_keys.add(key)
            self._sync_stick_with_keyboard(key)
            return True
        if key == 'tab':
            self.next_control_mode()
            return True
        if key in {'+', '='}:
            self.adjust_sensitivity(self.sensitivity_step)
            return True
        if key in {'-', '_'}:
            self.adjust_sensitivity(-self.sensitivity_step)
            return True
        if key in {'1', '2', '3'}:
            self.set_gait_index(int(key) - 1)
            return True
        return False

    def key_up(self, key: str) -> bool:
        """Apply a normalized key release and return whether state changed."""
        key = key.lower()
        if key not in MOTION_KEYS or key not in self.held_keys:
            return False
        self.held_keys.remove(key)
        self._sync_stick_with_keyboard(key)
        return True

    def adjust_sensitivity(self, delta: float):
        """Adjust keyboard emulation magnitude, re-applying held keys live."""
        self.sensitivity = clamp(
            self.sensitivity + delta,
            self.minimum_sensitivity,
            self.maximum_sensitivity,
        )
        self.sensitivity = round(self.sensitivity, 2)
        for stick, key_vectors in (
            (self.left_stick, LEFT_KEY_VECTORS),
            (self.right_stick, RIGHT_KEY_VECTORS),
        ):
            if not stick.dragging and self._any_keys_held(key_vectors):
                self._apply_keyboard(stick, key_vectors)

    # ----------------------------------------------------------------- pointer

    def set_left_stick(self, x: float, y: float):
        """Drive the left stick from a pointer drag (unit-circle clamped)."""
        self._pointer_set(self.left_stick, x, y)

    def release_left_stick(self):
        """End the left stick pointer drag (latches in body modes)."""
        self._pointer_release(self.left_stick, LEFT_KEY_VECTORS)

    def set_right_stick(self, x: float, y: float):
        """Drive the right stick from a pointer drag (unit-circle clamped)."""
        self._pointer_set(self.right_stick, x, y)

    def release_right_stick(self):
        """End the right stick pointer drag (latches in body modes)."""
        self._pointer_release(self.right_stick, RIGHT_KEY_VECTORS)

    def set_left_trigger(self, value: float):
        """Set left trigger slider value."""
        self.left_trigger = clamp(value, 0.0, 1.0)

    def set_right_trigger(self, value: float):
        """Set right trigger slider value."""
        self.right_trigger = clamp(value, 0.0, 1.0)

    # ------------------------------------------------------------ mode / gait

    @property
    def gait(self) -> str:
        """Return the currently selected gait name."""
        return self.gaits[self.gait_index]

    def set_gait_index(self, gait_index: int):
        """Set selected gait by index."""
        self.gait_index = max(0, min(len(self.gaits) - 1, gait_index))

    def set_control_mode(self, control_mode: ControlMode):
        """Switch control mode, preserving latched body pose per mode."""
        if control_mode == self._control_mode:
            return

        self._update_active_mode_fields()
        if self._control_mode in LATCHING_MODES:
            self._mode_stick_memory[self._control_mode] = (
                self.left_stick.value,
                self.right_stick.value,
            )
        else:
            # Leaving Walk: unlike a held gamepad stick, GUI input cannot be
            # carried across the switch, so walking must stop immediately.
            self.stride_direction = ZERO_VECTOR
            self.rotation_speed = 0.0

        self._control_mode = control_mode

        if control_mode in LATCHING_MODES:
            left, right = self._mode_stick_memory.get(control_mode, ((0.0, 0.0), (0.0, 0.0)))
            for stick, value in ((self.left_stick, left), (self.right_stick, right)):
                stick.set_axes(*value)
                stick.dragging = False
                stick.latched = True
        else:
            for stick, key_vectors in (
                (self.left_stick, LEFT_KEY_VECTORS),
                (self.right_stick, RIGHT_KEY_VECTORS),
            ):
                stick.dragging = False
                stick.latched = False
                self._apply_keyboard(stick, key_vectors)

    def next_control_mode(self):
        """Cycle through the same control modes as the joystick translator."""
        current_index = all_control_modes.index(self._control_mode)
        self.set_control_mode(all_control_modes[(current_index + 1) % len(all_control_modes)])

    # ---------------------------------------------------------------- commands

    def axes(self) -> VirtualAxes:
        """Return the merged keyboard and pointer controller axes."""
        return VirtualAxes(
            left_x=self.left_stick.x,
            left_y=self.left_stick.y,
            right_x=self.right_stick.x,
            right_y=self.right_stick.y,
            left_trigger=self.left_trigger,
            right_trigger=self.right_trigger,
        )

    def movement_command(self) -> MovementCommand:
        """Update the active mode's command fields and build the full command."""
        self._update_active_mode_fields()

        command = MovementCommand()
        command.gait_type = self.gait
        command.stride_direction = _vector3(self.stride_direction)
        command.rotation_speed = float(self.rotation_speed)
        command.body_translation = _vector3(self.body_translation)
        command.body_rotation = _vector3(self.body_rotation)
        return command

    def reset_motion_inputs(self):
        """Clear all latched motion inputs and command state."""
        self.held_keys.clear()
        self.left_stick.reset()
        self.right_stick.reset()
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.stride_direction = ZERO_VECTOR
        self.rotation_speed = 0.0
        self.body_translation = ZERO_VECTOR
        self.body_rotation = ZERO_VECTOR
        self._mode_stick_memory.clear()

    # ----------------------------------------------------------------- helpers

    def _update_active_mode_fields(self):
        """Drive the active mode's command fields from the current axes."""
        axes = self.axes()
        if self._control_mode == ControlMode.Walk:
            self.stride_direction = (axes.left_y, -axes.left_x, axes.left_trigger)
            self.rotation_speed = -axes.right_x
        elif self._control_mode == ControlMode.BodyPosition:
            self.body_translation = (axes.left_y, -axes.left_x, axes.right_y)
        elif self._control_mode == ControlMode.BodyRotation:
            self.body_rotation = (-axes.left_x, axes.left_y, -axes.right_x)

    def _pointer_set(self, stick: VirtualStick, x: float, y: float):
        stick.dragging = True
        stick.latched = False
        stick.set_axes(*clamp_vector(x, y))

    def _pointer_release(self, stick: VirtualStick, key_vectors: dict):
        stick.dragging = False
        if self._control_mode in LATCHING_MODES:
            stick.latched = True
        else:
            self._apply_keyboard(stick, key_vectors)

    def _sync_stick_with_keyboard(self, key: str):
        if key in LEFT_STICK_KEYS:
            stick, key_vectors = self.left_stick, LEFT_KEY_VECTORS
        else:
            stick, key_vectors = self.right_stick, RIGHT_KEY_VECTORS
        if stick.dragging:
            return
        if self._any_keys_held(key_vectors):
            self._apply_keyboard(stick, key_vectors)
        elif self._control_mode in LATCHING_MODES:
            # Last key released: hold position, same as a pointer release.
            stick.latched = True
        else:
            stick.set_axes(0.0, 0.0)

    def _apply_keyboard(self, stick: VirtualStick, key_vectors: dict):
        """Drive a stick from held keys; keyboard input overrides a latch."""
        x = 0.0
        y = 0.0
        for key, (key_x, key_y) in key_vectors.items():
            if key in self.held_keys:
                x += key_x
                y += key_y
        stick.set_axes(x * self.sensitivity, y * self.sensitivity)
        stick.latched = False

    def _any_keys_held(self, key_vectors: dict) -> bool:
        return any(key in self.held_keys for key in key_vectors)


def _vector3(value: tuple) -> Vector3:
    x, y, z = value
    return Vector3(x=float(x), y=float(y), z=float(z))
