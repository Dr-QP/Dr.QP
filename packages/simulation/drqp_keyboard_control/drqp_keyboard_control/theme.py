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

"""Color palette, font sizes, and shape constants for the GUI."""

from dataclasses import dataclass

Color = tuple[int, int, int]


@dataclass(frozen=True)
class Theme:
    """Visual style shared by every widget renderer."""

    background: Color = (21, 24, 28)

    text_primary: Color = (235, 239, 244)
    text_normal: Color = (203, 211, 219)
    text_muted: Color = (178, 187, 197)

    button_fill: Color = (47, 54, 61)
    button_selected: Color = (63, 132, 103)
    button_pressed: Color = (150, 80, 68)
    button_border: Color = (111, 126, 140)
    button_radius: int = 6

    checkbox_border: Color = (150, 163, 177)
    checkbox_border_pressed: Color = (210, 218, 226)
    checkbox_mark: Color = (86, 160, 133)
    checkbox_radius: int = 4

    stick_fill: Color = (38, 44, 51)
    stick_border: Color = (91, 104, 118)
    stick_axis: Color = (62, 70, 80)
    stick_knob: Color = (86, 160, 133)
    stick_knob_border: Color = (235, 239, 244)
    stick_knob_radius: int = 34

    trigger_track: Color = (45, 51, 58)
    trigger_fill: Color = (80, 130, 184)
    trigger_knob: Color = (235, 239, 244)
    trigger_knob_radius: int = 14

    panel_fill: Color = (34, 39, 45)
    panel_border: Color = (118, 132, 146)

    font_size: int = 28
    small_font_size: int = 22


DEFAULT_THEME = Theme()
