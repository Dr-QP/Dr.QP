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

from .utils import is_sphinx_build_no_videos, is_sphinx_build
from ipywidgets import interact
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib import pyplot as plt
from pathlib import Path


def animate_plot(
    _fig,
    _animate: callable,
    _frames,
    _interval=16,  # 60 fps
    _interactive=False,
    _save_animation_name=None,
    **interact_kwargs,
):
    if is_sphinx_build_no_videos():
        return

    anim = None

    plt.rcParams['animation.html'] = 'html5'

    if _interactive and not is_sphinx_build():
        with plt.ion():
            anim = interact(_animate, frame=(0, _frames), **interact_kwargs)
            plt.show()
    else:
        with plt.ioff():
            anim = FuncAnimation(_fig, _animate, frames=_frames, interval=_interval)
            display(anim)  # type: ignore # noqa: F821
            plt.close(_fig)

            if _save_animation_name is not None:
                animation_writer = FFMpegWriter(fps=24)
                anim.save(Path(_save_animation_name).with_suffix('.mp4'), writer=animation_writer)

    return anim
