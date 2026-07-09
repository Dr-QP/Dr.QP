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

from drqp_keyboard_control.layout import (
    Align,
    Box,
    clamp_vector,
    Column,
    Justify,
    RectSpec,
    Row,
    solve,
)
import pytest


def test_row_places_fixed_children_with_spacing():
    """Fixed-size children should stack left to right with spacing."""
    root = Row(
        children=[
            Box(key='a', width=100.0, height=30.0),
            Box(key='b', width=50.0, height=30.0),
        ],
        spacing=10.0,
    )

    rects = solve(root, 400.0, 100.0)

    assert rects['a'] == RectSpec(0.0, 0.0, 100.0, 30.0)
    assert rects['b'] == RectSpec(110.0, 0.0, 50.0, 30.0)


def test_column_places_fixed_children_with_spacing_and_padding():
    """Padding should inset children on both axes."""
    root = Column(
        children=[
            Box(key='a', width=100.0, height=30.0),
            Box(key='b', width=100.0, height=40.0),
        ],
        spacing=8.0,
        padding=16.0,
    )

    rects = solve(root, 400.0, 400.0)

    assert rects['a'] == RectSpec(16.0, 16.0, 100.0, 30.0)
    assert rects['b'] == RectSpec(16.0, 54.0, 100.0, 40.0)


def test_flex_children_share_leftover_space_by_weight():
    """Flex children should split leftover space proportionally."""
    root = Row(
        children=[
            Box(key='fixed', width=100.0, height=10.0),
            Box(key='one', flex=1.0),
            Box(key='three', flex=3.0),
        ],
        spacing=10.0,
    )

    rects = solve(root, 520.0, 50.0)

    assert rects['fixed'].width == pytest.approx(100.0)
    assert rects['one'].width == pytest.approx(100.0)
    assert rects['three'].width == pytest.approx(300.0)
    assert rects['one'].x == pytest.approx(110.0)
    assert rects['three'].x == pytest.approx(220.0)


def test_explicit_main_axis_size_beats_flex():
    """Explicit size on the main axis should override flex sizing."""
    root = Row(
        children=[
            Box(key='fixed_flex', width=120.0, height=10.0, flex=1.0),
            Box(key='flex_only', flex=1.0),
        ],
        spacing=10.0,
    )

    rects = solve(root, 410.0, 50.0)

    assert rects['fixed_flex'].width == pytest.approx(120.0)
    assert rects['flex_only'].width == pytest.approx(280.0)
    assert rects['flex_only'].x == pytest.approx(130.0)


def test_flex_column_fills_viewport_height():
    """A flex child in a column should absorb all remaining height."""
    root = Column(
        children=[
            Box(key='header', height=40.0),
            Box(key='body', flex=1.0),
            Box(key='footer', height=60.0),
        ],
    )

    rects = solve(root, 300.0, 640.0)

    assert rects['body'].y == pytest.approx(40.0)
    assert rects['body'].height == pytest.approx(540.0)
    assert rects['footer'].y == pytest.approx(580.0)


def test_align_stretch_fills_cross_axis():
    """Stretch alignment should expand children across the container."""
    root = Row(
        children=[Box(key='a', width=50.0)],
        align=Align.stretch,
    )

    rects = solve(root, 200.0, 120.0)

    assert rects['a'].height == pytest.approx(120.0)


def test_align_center_and_end_position_cross_axis():
    """Center/end alignment should offset children on the cross axis."""
    centered = Row(children=[Box(key='c', width=50.0, height=40.0)], align=Align.center)
    ended = Row(children=[Box(key='e', width=50.0, height=40.0)], align=Align.end)

    center_rects = solve(centered, 200.0, 100.0)
    end_rects = solve(ended, 200.0, 100.0)

    assert center_rects['c'].y == pytest.approx(30.0)
    assert end_rects['e'].y == pytest.approx(60.0)


def test_justify_center_and_end_distribute_leftover_main_axis():
    """Justify should place fixed-size children within leftover space."""

    def children():
        return [Box(key='a', width=60.0, height=10.0)]

    center = solve(Row(children=children(), justify=Justify.center), 200.0, 50.0)
    end = solve(Row(children=children(), justify=Justify.end), 200.0, 50.0)

    assert center['a'].x == pytest.approx(70.0)
    assert end['a'].x == pytest.approx(140.0)


def test_nested_containers_compose():
    """Nested rows inside columns should inherit their parent slots."""
    root = Column(
        children=[
            Row(
                key='toolbar',
                children=[
                    Box(key='left', width=40.0, height=20.0),
                    Box(key='right', width=40.0, height=20.0),
                ],
                spacing=4.0,
                height=28.0,
            ),
            Box(key='content', flex=1.0),
        ],
        padding=10.0,
    )

    rects = solve(root, 300.0, 200.0)

    assert rects['toolbar'] == RectSpec(10.0, 10.0, 280.0, 28.0)
    assert rects['left'] == RectSpec(10.0, 10.0, 40.0, 20.0)
    assert rects['right'] == RectSpec(54.0, 10.0, 40.0, 20.0)
    assert rects['content'].y == pytest.approx(38.0)
    assert rects['content'].height == pytest.approx(152.0)


def test_container_measures_intrinsic_size_from_children():
    """Containers without explicit size should measure their content."""
    row = Row(
        children=[
            Box(width=40.0, height=20.0),
            Box(width=60.0, height=30.0),
        ],
        spacing=10.0,
        padding=5.0,
    )

    assert row.measure() == (120.0, 40.0)


def test_rect_contains_and_center():
    """The rect helper should expose hit testing and center math."""
    rect = RectSpec(10.0, 20.0, 100.0, 50.0)

    assert rect.contains((10.0, 20.0)) is True
    assert rect.contains((110.0, 70.0)) is True
    assert rect.contains((111.0, 70.0)) is False
    assert rect.center == (60.0, 45.0)


def test_clamp_vector_limits_to_unit_circle():
    """The vector helper should preserve direction while limiting magnitude."""
    x, y = clamp_vector(3.0, 4.0)

    assert x == pytest.approx(0.6)
    assert y == pytest.approx(0.8)
    assert clamp_vector(0.3, -0.4) == (0.3, -0.4)
