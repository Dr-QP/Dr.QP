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
Minimal flexbox-style layout engine.

Pure Python and Pygame-free so layouts can be unit tested. A layout is a tree
of ``Row``/``Column`` containers and ``Box`` leaves. Leaves (and containers)
may carry a ``key``; ``solve()`` computes the final rectangles for the given
viewport and returns them indexed by key.

Sizing model per node:
- ``width``/``height``: fixed size on that dimension when set.
- ``flex``: share of the leftover main-axis space in the parent container
  (after fixed-size children and spacing are subtracted).
- Otherwise a node falls back to its measured (intrinsic) size.

Container extras: ``spacing`` between children, uniform ``padding``,
``justify`` for main-axis placement of leftover space, and ``align`` for
cross-axis placement (``stretch`` fills the cross axis).
"""

from dataclasses import dataclass, field
import enum
import math


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
        """Return whether the point lies inside the rectangle."""
        px, py = pos
        return self.x <= px <= self.x + self.width and self.y <= py <= self.y + self.height

    @property
    def center(self) -> tuple[float, float]:
        """Return the rectangle center point."""
        return (self.x + self.width / 2.0, self.y + self.height / 2.0)


class Justify(enum.Enum):
    """Main-axis placement of children when leftover space remains."""

    start = enum.auto()
    center = enum.auto()
    end = enum.auto()


class Align(enum.Enum):
    """
    Cross-axis placement of a child inside a container.

    ``stretch`` (the default, as in CSS flexbox) fills the cross axis unless
    the child declares an explicit cross-axis size, which is then kept.
    """

    start = enum.auto()
    center = enum.auto()
    end = enum.auto()
    stretch = enum.auto()


@dataclass
class Box:
    """Leaf layout node, typically bound to a widget through its key."""

    key: str | None = None
    width: float | None = None
    height: float | None = None
    flex: float = 0.0

    def measure(self) -> tuple[float, float]:
        """Return the intrinsic (width, height) of this node."""
        return (self.width or 0.0, self.height or 0.0)

    def arrange(self, rect: RectSpec, rects: dict[str, RectSpec]):
        """Assign the final rectangle, recording it under this node's key."""
        if self.key is not None:
            rects[self.key] = rect


@dataclass
class _Container(Box):
    """Shared row/column behavior; subclasses select the main axis."""

    children: list[Box] = field(default_factory=list)
    spacing: float = 0.0
    padding: float = 0.0
    justify: Justify = Justify.start
    align: Align = Align.stretch

    # Index into a (width, height) tuple: 0 lays children out horizontally.
    main_axis: int = 0

    def measure(self) -> tuple[float, float]:
        """Measure content plus spacing and padding, fixed sizes override."""
        sizes = [child.measure() for child in self.children]
        main = sum(size[self.main_axis] for size in sizes)
        if self.children:
            main += self.spacing * (len(self.children) - 1)
        cross = max((size[1 - self.main_axis] for size in sizes), default=0.0)

        measured = [0.0, 0.0]
        measured[self.main_axis] = main + 2.0 * self.padding
        measured[1 - self.main_axis] = cross + 2.0 * self.padding
        return (
            self.width if self.width is not None else measured[0],
            self.height if self.height is not None else measured[1],
        )

    def arrange(self, rect: RectSpec, rects: dict[str, RectSpec]):
        """Distribute the content rect between children along the main axis."""
        super().arrange(rect, rects)

        content = RectSpec(
            rect.x + self.padding,
            rect.y + self.padding,
            max(0.0, rect.width - 2.0 * self.padding),
            max(0.0, rect.height - 2.0 * self.padding),
        )
        content_size = (content.width, content.height)
        main_extent = content_size[self.main_axis]
        cross_extent = content_size[1 - self.main_axis]

        main_sizes = self._resolve_main_sizes(main_extent)

        used = sum(main_sizes) + self.spacing * max(0, len(self.children) - 1)
        offset = self._justify_offset(main_extent - used)

        cursor = (content.x, content.y)[self.main_axis]
        cursor += offset
        for child, main_size in zip(self.children, main_sizes):
            cross_size, cross_offset = self._align_cross(child, cross_extent)

            origin = [0.0, 0.0]
            origin[self.main_axis] = cursor
            origin[1 - self.main_axis] = (content.x, content.y)[1 - self.main_axis] + cross_offset
            size = [0.0, 0.0]
            size[self.main_axis] = main_size
            size[1 - self.main_axis] = cross_size

            child.arrange(RectSpec(origin[0], origin[1], size[0], size[1]), rects)
            cursor += main_size + self.spacing

    def _resolve_main_sizes(self, main_extent: float) -> list[float]:
        fixed_sizes = [child.measure()[self.main_axis] for child in self.children]
        total_spacing = self.spacing * max(0, len(self.children) - 1)
        flex_children = [
            child
            for child in self.children
            if child.flex and self._explicit_main_axis_size(child) is None
        ]
        total_flex = sum(child.flex for child in flex_children)
        leftover = main_extent - total_spacing
        leftover -= sum(
            size
            for child, size in zip(self.children, fixed_sizes)
            if not child.flex or self._explicit_main_axis_size(child) is not None
        )
        leftover = max(0.0, leftover)

        sizes = []
        for child, fixed_size in zip(self.children, fixed_sizes):
            if self._explicit_main_axis_size(child) is not None:
                sizes.append(fixed_size)
            elif child.flex and total_flex:
                sizes.append(leftover * child.flex / total_flex)
            else:
                sizes.append(fixed_size)
        return sizes

    def _explicit_main_axis_size(self, child: Box) -> float | None:
        return child.width if self.main_axis == 0 else child.height

    def _justify_offset(self, leftover: float) -> float:
        leftover = max(0.0, leftover)
        if self.justify == Justify.center:
            return leftover / 2.0
        if self.justify == Justify.end:
            return leftover
        return 0.0

    def _align_cross(self, child: Box, cross_extent: float) -> tuple[float, float]:
        if self.align == Align.stretch:
            explicit = child.height if self.main_axis == 0 else child.width
            if explicit is None:
                return cross_extent, 0.0
            return min(explicit, cross_extent), 0.0
        cross_size = min(child.measure()[1 - self.main_axis], cross_extent)
        if self.align == Align.center:
            return cross_size, (cross_extent - cross_size) / 2.0
        if self.align == Align.end:
            return cross_size, cross_extent - cross_size
        return cross_size, 0.0


@dataclass
class Row(_Container):
    """Container that lays children out left to right."""

    main_axis: int = 0


@dataclass
class Column(_Container):
    """Container that lays children out top to bottom."""

    main_axis: int = 1


def solve(root: Box, width: float, height: float) -> dict[str, RectSpec]:
    """Lay out the tree in the given viewport and return rects by node key."""
    rects: dict[str, RectSpec] = {}
    root.arrange(RectSpec(0.0, 0.0, width, height), rects)
    return rects
