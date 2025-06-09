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

# 3. Generating gaits

The next step in developing the robot kinematics is to generate various gaits. First, lets familiarize ourselves with the concept of gaits. Gait is a coordinated sequence of movements of the robot legs and body to achieve smooth locomotion.

The gait cycle of a hexapod robot refers to the sequential movement of its six legs to achieve locomotion. It consists of two main phases for each leg:

 1. Stance Phase – The leg is in contact with the ground, providing support and propulsion as it moves backward relative to the body.
 2. Swing Phase – The leg lifts off the ground, moves forward, and prepares for the next stance phase.

To get an idea of the variance and complexity between gaits, Fig 1. shows four typical gaits. The six legs, right and left hind, middle and front, indicated as RH, RM, RF, LH, LM, and LF. Stance phase is shown in white, swing phase in black.

![Hexapod gaits](hexapod_gaits.png)

Fig 1. Four typical hexapodal gaits, depicting each of the six legs as either supporting (white) or recovering (black). Image source: (Chen et al. 2012) [borrowed from hexapodrobot.weebly.com](https://hexapodrobot.weebly.com/the-legs.html)

The three most common gaits for hexapods are the Wave, Transition (Ripple) and Tripod gaits. These are the three gaits we are going to implement in this notebook. Wave gait is the simplest and most stable as robot lifts only one leg at a time, it makes it the slowest one as well. Transition (ripple) gait is an improvement over the Wave gait as it allows to lift two legs at a time, with the half-phase shift. This makes it faster, at the slight cost of stability. Finally, the Tripod gait is the fastest and least stable one as it lifts three legs at a time.

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

+++ {"tags": ["remove-cell"]}

The next step is configuring matplotlib backend. Widget backend allows to interact with the plots in the notebook and is supported in Google Colab and VSCode. SVG format is used for the plots to make them look good in the hosted sphinx documentation.

```{code-cell} ipython3
:tags: [remove-cell]

%config InlineBackend.figure_formats = ['svg']
%matplotlib widget

import matplotlib.pyplot as plt
from plotting import display_and_close

plt.ioff()  # this is equivalent to using inline backend, but figures have to be displayed manually
```

## Parametric gait generator

The easiest generic way to implement gait generation is to use a parametric function. The parametric function is defined in the leg's local coordinate system, with the origin at the leg's base and the X axis pointing forward. The function takes a single parameter, the phase of the gait cycle, and returns the leg's offset in the local coordinate system at that phase.

The parametric function is defined as a piecewise function that describes the leg's movement in the swing and stance phases. In the swing phase, the leg moves forward in the X direction and up in the Z direction. In the stance phase, the leg moves backward in the X direction and stays at the ground level (Z=0).

The swing and stance phases are defined by the `swing_duration` parameter, which is the fraction of the gait cycle that the leg is in the air. The swing phase starts at the beginning of the gait cycle and ends at `swing_duration`. The stance phase starts at `swing_duration` and ends at the end of the gait cycle.

The parametric function is defined as follows:

```{code-cell} ipython3
:tags: [remove-cell]

import jupyter_utils

jupyter_utils.display_file(
    'parametric_gait_generator.py',
    start_after='# Parametric function - START',
    end_before='# Parametric function - END',
)
```

```{literalinclude} parametric_gait_generator.py
:start-after: '# Parametric function - START'
:end-before: '# Parametric function - END'
```

With this function in place, we can now implement the gait generators for the three gaits. The gait generators are defined in the `gaits` dictionary in the `ParametricGaitGenerator` class. The keys of the dictionary are the gait types, and the values are the parameters of the gait. The parameters include the swing duration and the swing phase start offsets for each leg. The swing phase start offsets define the phase at which the leg starts the swing phase.

```{code-cell} ipython3
:tags: [remove-cell]

jupyter_utils.display_file(
    'parametric_gait_generator.py',
    start_after='# Gait params - START',
    end_before='# Gait params - END',
)
```

```{literalinclude} parametric_gait_generator.py
:start-after: '# Gait params - START'
:end-before: '# Gait params - END'
```

The base class `GaitGenerator` defines the basic interface for the gait generators. The `get_offsets_at_phase` method returns the leg offsets at a specific phase of the gait cycle. The `get_offsets_at_phase_for_leg` method returns the leg offset for a specific leg at a specific phase.

The `visualize_continuous` and `visualize_continuous_in_3d` methods are used to visualize the gait in 2D and 3D.

```{code-cell} ipython3
from parametric_gait_generator import ParametricGaitGenerator, GaitType
from models import HexapodModel
from plotting import GaitsVisualizer

hexapod = HexapodModel()
hexapod.forward_kinematics(0, -25, 110)

gait_gen = ParametricGaitGenerator(step_length=120, step_height=50)

visualizer = GaitsVisualizer()
```

```{code-cell} ipython3
gait_gen.current_gait = GaitType.wave
visualizer.visualize_continuous(gait_gen, _steps=100)
_ = visualizer.visualize_continuous_in_3d(gait_gen, _steps=100)
```

```{code-cell} ipython3
gait_gen.current_gait = GaitType.ripple
visualizer.visualize_continuous(gait_gen, _steps=100)
_ = visualizer.visualize_continuous_in_3d(gait_gen, _steps=100)
```

```{code-cell} ipython3
gait_gen.current_gait = GaitType.tripod
visualizer.visualize_continuous(gait_gen, _steps=100)
_ = visualizer.visualize_continuous_in_3d(gait_gen, _steps=100)
```
