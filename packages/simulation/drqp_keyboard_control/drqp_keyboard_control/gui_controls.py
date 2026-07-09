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
Pygame-free GUI widget models and pointer routing.

Widgets expose a uniform pointer interface (``hit_test``/``press``/``drag``/
``release``) plus value callables the renderer reads from, so behavior is
fully unit testable. Rectangles are assigned from the layout engine through
each widget's ``layout_key``.
"""

from dataclasses import dataclass, field
import math
from typing import Callable

from drqp_keyboard_control.layout import clamp, RectSpec

# Vertical room reserved inside a stick rect for its name (above) and its
# axis readout (below); the interactive circle fills the rest.
STICK_TEXT_MARGIN = 40.0

# Trigger geometry: label strip above the track, track thickness across it.
TRIGGER_LABEL_STRIP = 26.0
TRIGGER_TRACK_THICKNESS = 34.0


def _zero_rect() -> RectSpec:
    return RectSpec(0.0, 0.0, 0.0, 0.0)


def _never_selected() -> bool:
    return False


@dataclass(kw_only=True)
class Control:
    """Base widget: a labeled rectangle with a uniform pointer interface."""

    label: str
    layout_key: str
    rect: RectSpec = field(default_factory=_zero_rect)

    def hit_test(self, pos: tuple[float, float]) -> bool:
        """Return whether the point interacts with this widget."""
        return self.rect.contains(pos)

    def press(self, pos: tuple[float, float]):
        """Handle a pointer press inside the widget."""

    def drag(self, pos: tuple[float, float]):
        """Handle pointer motion while this widget holds the capture."""

    def release(self):
        """Handle the pointer release ending the capture."""


@dataclass(kw_only=True)
class LabelControl(Control):
    """Non-interactive text display."""

    text: Callable[[], str]

    def hit_test(self, pos: tuple[float, float]) -> bool:
        """Labels never take pointer input."""
        return False


@dataclass(kw_only=True)
class ButtonControl(Control):
    """Clickable GUI button firing its action on press."""

    action: Callable[[], None]
    selected: Callable[[], bool] = _never_selected
    pressed: bool = False

    def press(self, pos: tuple[float, float]):
        """Fire the action and show the pressed state until release."""
        self.pressed = True
        self.action()

    def release(self):
        """Clear the pressed visual state."""
        self.pressed = False


@dataclass(kw_only=True)
class CheckboxControl(ButtonControl):
    """Toggle rendered as a checkbox; behaves exactly like a button."""


@dataclass(kw_only=True)
class StickControl(Control):
    """Draggable circular virtual thumb stick."""

    on_move: Callable[[float, float], None]
    on_release: Callable[[], None]
    value: Callable[[], tuple[float, float]]
    dragging: bool = False

    @property
    def center(self) -> tuple[float, float]:
        """Return the stick center in window coordinates."""
        return self.rect.center

    @property
    def radius(self) -> float:
        """Return the stick radius derived from the layout rect."""
        return max(
            10.0,
            min(self.rect.width / 2.0, self.rect.height / 2.0 - STICK_TEXT_MARGIN),
        )

    def hit_test(self, pos: tuple[float, float]) -> bool:
        """Sticks react only within their circle, not the whole rect."""
        center_x, center_y = self.center
        return math.hypot(pos[0] - center_x, pos[1] - center_y) <= self.radius

    def press(self, pos: tuple[float, float]):
        """Start dragging the stick knob."""
        self.dragging = True
        self.drag(pos)

    def drag(self, pos: tuple[float, float]):
        """Report the knob position as normalized axes (y up)."""
        center_x, center_y = self.center
        x = (pos[0] - center_x) / self.radius
        y = -(pos[1] - center_y) / self.radius
        self.on_move(x, y)

    def release(self):
        """End the drag; latching policy is decided by the state model."""
        self.dragging = False
        self.on_release()


@dataclass(kw_only=True)
class TriggerControl(Control):
    """Horizontal or vertical trigger slider that keeps its value."""

    on_change: Callable[[float], None]
    value: Callable[[], float]
    dragging: bool = False

    @property
    def vertical(self) -> bool:
        """Return whether the slider runs bottom-to-top."""
        return self.rect.height > self.rect.width

    @property
    def track_rect(self) -> RectSpec:
        """Return the slider track: below the label strip, centered across."""
        track = RectSpec(
            self.rect.x,
            self.rect.y + TRIGGER_LABEL_STRIP,
            self.rect.width,
            max(1.0, self.rect.height - TRIGGER_LABEL_STRIP),
        )
        if self.vertical:
            track.x += (track.width - TRIGGER_TRACK_THICKNESS) / 2.0
            track.width = TRIGGER_TRACK_THICKNESS
        else:
            track.y += (track.height - TRIGGER_TRACK_THICKNESS) / 2.0
            track.height = TRIGGER_TRACK_THICKNESS
        return track

    def press(self, pos: tuple[float, float]):
        """Start dragging the slider knob."""
        self.dragging = True
        self.drag(pos)

    def drag(self, pos: tuple[float, float]):
        """Report the slider value for the pointer position."""
        track = self.track_rect
        if self.vertical:
            value = 1.0 - ((pos[1] - track.y) / track.height)
        else:
            value = (pos[0] - track.x) / track.width
        self.on_change(clamp(value, 0.0, 1.0))

    def release(self):
        """End the drag; sliders keep their last value."""
        self.dragging = False


class PointerRouter:
    """Dispatch pointer events to widgets with single-capture semantics."""

    def __init__(self, controls: list[Control]):
        self.controls = controls
        self._active: Control | None = None

    @property
    def active(self) -> Control | None:
        """Return the widget currently holding the pointer capture."""
        return self._active

    def press(self, pos: tuple[float, float]) -> bool:
        """Send the press to the first hit widget and capture it."""
        for control in self.controls:
            if control.hit_test(pos):
                self._active = control
                control.press(pos)
                return True
        return False

    def move(self, pos: tuple[float, float]):
        """Forward pointer motion to the captured widget only."""
        if self._active is not None:
            self._active.drag(pos)

    def release(self):
        """Release the capture, notifying the captured widget."""
        if self._active is not None:
            self._active.release()
            self._active = None
