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

import ctypes
import ctypes.util
import logging
from pathlib import Path


logger = logging.getLogger(__name__)


def set_sdl_window_always_on_top(window_id: int | None, enabled: bool) -> bool:
    """Set the Pygame SDL window topmost flag when SDL exposes the API."""
    if window_id is None:
        return False

    for library_name in sdl_library_candidates():
        if _set_sdl_window_always_on_top_with_library(library_name, window_id, enabled):
            return True

    return False


def _set_sdl_window_always_on_top_with_library(
    library_name: str,
    window_id: int,
    enabled: bool,
) -> bool:
    try:
        sdl = ctypes.CDLL(library_name)
        get_window = sdl.SDL_GetWindowFromID
        get_window.argtypes = [ctypes.c_uint32]
        get_window.restype = ctypes.c_void_p
        window = get_window(ctypes.c_uint32(window_id))
        if not window:
            return False

        set_always_on_top = sdl.SDL_SetWindowAlwaysOnTop
        set_always_on_top.argtypes = [ctypes.c_void_p, ctypes.c_int]
        set_always_on_top.restype = None
        set_always_on_top(window, 1 if enabled else 0)
    except (AttributeError, OSError, TypeError, ValueError):
        return False

    return True


def sdl_library_candidates() -> list[str]:
    """Return likely SDL library paths, including Pygame-bundled binaries."""
    candidates = [
        ctypes.util.find_library('SDL2'),
        ctypes.util.find_library('SDL2-2.0'),
    ]

    try:
        import pygame

        pygame_dir = Path(pygame.__file__).resolve().parent
        search_dirs = [
            pygame_dir,
            pygame_dir / '.dylibs',
            pygame_dir.parent / '.dylibs',
            pygame_dir / 'pygame.libs',
            pygame_dir.parent / 'pygame.libs',
        ]
        patterns = (
            'libSDL2*.dylib',
            'SDL2*.dylib',
            'libSDL2*.so*',
            'SDL2*.dll',
        )
        for search_dir in search_dirs:
            if not search_dir.is_dir():
                continue
            for pattern in patterns:
                candidates.extend(str(path) for path in search_dir.glob(pattern))
    except (ImportError, OSError):
        logger.debug("Unable to inspect pygame-bundled SDL libraries", exc_info=True)

    unique_candidates = []
    for candidate in candidates:
        if candidate and candidate not in unique_candidates:
            unique_candidates.append(candidate)
    return unique_candidates
