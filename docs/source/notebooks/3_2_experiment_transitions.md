---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.17.2
kernelspec:
  display_name: .venv
  language: python
  name: python3
---

# 3.2 Appendix: Transitions

This notebook is an appendix to the `3_generating_gaits.ipynb` notebook. It contains some experiments with transitions in and out of gaits.

+++ {"tags": ["remove-cell"]}

## Setting up the Jupyter notebook for experimentation

The next couple of cells are designated to the setup of the notebook environment.

The first step is to enable live python modules reloading, so changes in the python code of imported files are immediately reflected in the notebook without restarting the kernel.

```{code-cell} ipython3
:tags: [remove-cell]

# Enable python modules live reloading
%load_ext autoreload
%autoreload 2
```

The next step is configuring matplotlib backend. Widget backend allows to interact with the plots in the notebook and is supported in Google Colab and VSCode. SVG format is used for the plots to make them look good in the hosted sphinx documentation.

```{code-cell} ipython3
:tags: [remove-cell]

%config InlineBackend.figure_formats = ['svg']
%matplotlib widget

import matplotlib.pyplot as plt

plt.ioff()  # this is equivalent to using inline backend, but figures have to be displayed manually
```

This gives a nice forward locomotive gait. However some changes are needed to the generated gait. Right now it starts with legs on the ground, but with maxed out X offsets. To mitigate this we need to introduce a transition stage that will take legs from whatever position they are in to the starting position.

A good starting point for the transition is a 0.25 phase mark where all legs have zero offsets in X axis, however group A is lifted up. In order to start from all the legs on the ground we need to compress Z phase to quarter of the original cycle.

So here is the plan for transition stage:

 1. It runs for 0.25 of the phase
 2. We start with X cycle at 0.25
 3. We start Z cycle 0, but compress first 0.5 of it to 0.25.
 4. At 0.5 both cycles sync up and cycle continues till 1.0
 5. Then we start the full cycle.

Transition out of gait is similar, but starts at 0 and ends at 0.25 or X and Z cycles is compressed to 0.25.

Below is the implementation of the transition. This kind of code works for the animation and is suitable for tripod gait, but with increase of gait complexity and with joystick style controls it would be impossible to implement it this way. We are going to explore a different approach later in this notebook.

```{code-cell} ipython3
from models import HexapodModel
from parametric_gait_generator import GaitType, ParametricGaitGenerator

hexapod = HexapodModel()
hexapod.forward_kinematics(0, -25, 110)

gait_gen = ParametricGaitGenerator(step_length=120, step_height=50)
gait_gen.current_gait = GaitType.tripod
```

```{code-cell} ipython3
from geometry import Point3D
import numpy as np
from plotting import animate_plot, plot_hexapod, update_hexapod_plot


def animate_hexapod_gait_with_transitions(
    hexapod: HexapodModel,
    gaits_gen,
    interactive=False,
    skip=False,
    total_steps=60,
    interval=16,
    view_elev=7.0,
    view_azim=-112,
    repeat=2,
):
    if skip:
        return
    transition_time_in = 0.75
    transition_steps_in = int(total_steps * transition_time_in)
    transition_steps_intro = int(total_steps * 0.25)
    transition_steps_rest = int(transition_steps_in - transition_steps_intro)

    phase_in_x_steps = np.concatenate(
        (
            np.linspace(0.25, 0.5, transition_steps_intro),
            np.linspace(0.5, 1.0, transition_steps_rest),
        )
    )
    phase_in_z_steps = np.concatenate(
        (
            np.linspace(0.0, 0.5, transition_steps_intro),
            np.linspace(0.5, 1.0, transition_steps_rest),
        )
    )

    transition_time_out = 0.25
    transition_steps_out = int(total_steps * transition_time_out)
    phase_out_x_steps = np.linspace(0.0, 0.25, transition_steps_out + 1)
    phase_out_z_steps = np.linspace(0.0, 0.5, transition_steps_out + 1)

    transition_steps_end = total_steps * repeat + transition_steps_in

    total_frames = total_steps * repeat + transition_steps_in + transition_steps_out

    leg_tips = [leg.tibia_end.copy() for leg in hexapod.legs]

    def set_pose(step):
        if step < transition_steps_in:
            phase_x = phase_in_x_steps[step]
            phase_z = phase_in_z_steps[step]

            offsets = {}
            for leg in hexapod.legs:
                off = gaits_gen.get_offsets_at_phase_for_leg(leg.label, phase_x)
                offsets[leg.label] = Point3D([off.x, 0, 0])

            for leg in hexapod.legs:
                off = gaits_gen.get_offsets_at_phase_for_leg(leg.label, phase_z)
                offsets[leg.label] += Point3D([0, 0, off.z])

            for leg, leg_tip in zip(hexapod.legs, leg_tips):
                leg.move_to(leg_tip + offsets[leg.label])
        elif step < transition_steps_end:
            step = step - transition_steps_in
            step = step % total_steps  # handle repeats
            phase = step / total_steps  # interpolation phase
            for leg, leg_tip in zip(hexapod.legs, leg_tips):
                offsets = gaits_gen.get_offsets_at_phase_for_leg(leg.label, phase)
                leg.move_to(leg_tip + offsets)

        else:
            end_step = step - transition_steps_end
            phase_x = phase_out_x_steps[end_step]
            phase_z = phase_out_z_steps[end_step]

            offsets = {}
            for leg in hexapod.legs:
                off = gaits_gen.get_offsets_at_phase_for_leg(leg.label, phase_x)
                offsets[leg.label] = Point3D([off.x, 0, 0])

            for leg in hexapod.legs:
                off = gaits_gen.get_offsets_at_phase_for_leg(leg.label, phase_z)
                offsets[leg.label] += Point3D([0, 0, off.z])

            for leg, leg_tip in zip(hexapod.legs, leg_tips):
                leg.move_to(leg_tip + offsets[leg.label])

    fig, ax, plot_data = plot_hexapod(hexapod)
    ax.view_init(elev=view_elev, azim=view_azim)

    def update(frame=0):
        set_pose(frame)
        update_hexapod_plot(hexapod, plot_data)
        fig.canvas.draw_idle()

    animate_plot(
        fig,
        update,
        _interactive=interactive,
        _frames=total_frames,
        _interval=interval,
    )


hexapod = HexapodModel()
hexapod.forward_kinematics(0, -25, 110)

anim = animate_hexapod_gait_with_transitions(
    hexapod, gait_gen, interactive=True, skip=False, view_elev=25
)
```
