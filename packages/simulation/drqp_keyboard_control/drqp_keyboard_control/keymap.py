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

"""Translation of Pygame key codes into the normalized key names of the model."""


def build_key_map(pygame) -> dict[int, str]:
    """Map Pygame key codes to the normalized key names GuiControlState expects."""
    return {
        pygame.K_w: 'w',
        pygame.K_a: 'a',
        pygame.K_s: 's',
        pygame.K_d: 'd',
        pygame.K_b: 'b',
        pygame.K_UP: 'up',
        pygame.K_DOWN: 'down',
        pygame.K_LEFT: 'left',
        pygame.K_RIGHT: 'right',
        pygame.K_TAB: 'tab',
        pygame.K_1: '1',
        pygame.K_2: '2',
        pygame.K_3: '3',
        pygame.K_PLUS: '+',
        pygame.K_EQUALS: '=',
        pygame.K_MINUS: '-',
        pygame.K_UNDERSCORE: '_',
        pygame.K_SPACE: 'space',
        pygame.K_ESCAPE: 'esc',
        pygame.K_DELETE: 'delete',
        pygame.K_BACKSPACE: 'backspace',
    }
