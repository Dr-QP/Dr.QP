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

from dataclasses import dataclass
import math
from typing import Callable


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Clamp value into inclusive range."""
    return max(minimum, min(maximum, value))


def clamp_vector(x: float, y: float) -> tuple[float, float]:
    """Clamp a 2D vector into the unit circle."""
    magnitude = math.hypot(x, y)
    if magnitude <= 1.0:
        return (x, y)
    return (x / magnitude, y / magnitude)


@dataclass
class RectSpec:
    """Small rectangle helper that keeps most GUI math Pygame-free."""

    x: float
    y: float
    width: float
    height: float

    def contains(self, pos: tuple[float, float]) -> bool:
        px, py = pos
        return self.x <= px <= self.x + self.width and self.y <= py <= self.y + self.height

    @property
    def center(self) -> tuple[float, float]:
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


@dataclass
class StickControl:
    """Draggable circular virtual thumb stick."""

    name: str
    center: tuple[float, float]
    radius: float
    setter: Callable[[float, float], None]
    releaser: Callable[[], None]
    dragging: bool = False

    def hit_test(self, pos: tuple[float, float]) -> bool:
        return math.hypot(pos[0] - self.center[0], pos[1] - self.center[1]) <= self.radius

    def begin_drag(self, pos: tuple[float, float]):
        self.dragging = True
        self.update_drag(pos)

    def update_drag(self, pos: tuple[float, float]):
        x = (pos[0] - self.center[0]) / self.radius
        y = -(pos[1] - self.center[1]) / self.radius
        self.setter(*clamp_vector(x, y))

    def end_drag(self):
        self.dragging = False
        self.releaser()


@dataclass
class TriggerControl:
    """Horizontal or vertical trigger slider."""

    name: str
    rect: RectSpec
    setter: Callable[[float], None]
    dragging: bool = False

    def begin_drag(self, pos: tuple[float, float]):
        if not self.rect.contains(pos):
            return
        self.dragging = True
        self.update_drag(pos)

    def update_drag(self, pos: tuple[float, float]):
        if self.rect.height > self.rect.width:
            value = 1.0 - ((pos[1] - self.rect.y) / self.rect.height)
        else:
            value = (pos[0] - self.rect.x) / self.rect.width
        self.setter(clamp(value, 0.0, 1.0))

    def end_drag(self):
        self.dragging = False


@dataclass
class ButtonControl:
    """Clickable GUI button."""

    label: str
    rect: RectSpec
    action: Callable[[], None]
    selected: Callable[[], bool] = lambda: False
    pressed: bool = False

    def click(self, pos: tuple[float, float]) -> bool:
        if not self.rect.contains(pos):
            return False
        self.pressed = True
        self.action()
        return True

    def release(self):
        self.pressed = False


@dataclass
class CheckboxControl:
    """Clickable checkbox GUI control."""

    label: str
    rect: RectSpec
    action: Callable[[], None]
    selected: Callable[[], bool]
    pressed: bool = False

    def click(self, pos: tuple[float, float]) -> bool:
        if not self.rect.contains(pos):
            return False
        self.pressed = True
        self.action()
        return True

    def release(self):
        self.pressed = False
