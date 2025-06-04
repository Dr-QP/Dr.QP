---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.17.1
kernelspec:
  display_name: .venv
  language: python
  name: python3
---

# 2. Matrix based transforms

In the last notebook `1_getting_started_with_robot_ik.ipynb` we have familiarized ourselves with the concept of Forward and Inverse Kinematics (IK) and implemented a simple IK solver for a 3DOF leg using trigonometry. In this notebook we will use SciPy to implement a more advanced IK solver using matrix transformations.

SciPy, a Python library built upon NumPy, provides a wide array of mathematical algorithms and functions that are valuable for robotics, including linear algebra, which is of interest here.

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

## Affine transforms

The first step is to convert our forward kinematics code from custom Points and hand crafted function to use matrix transforms for rotation and translation.

Alongside the conversion, we are going to bring all the algorithms to work in full 3D space, not just 2D projections.

As a first step in conversion, let's replace hand crafted rotations with rotation matrices.

```{code-cell} ipython3
from geometry import Leg3D, Line3D, Point3D
import numpy as np
from scipy.spatial.transform import Rotation as R


def forward_kinematics(
    coxa_length,
    femur_length,
    tibia_length,
    alpha,
    beta,
    gamma,
    start_height=2,
    body_length=5,
):
    # Define initial points in local coordinates
    start_point = Point3D([0, start_height, 0])
    body_local = Point3D([body_length, 0, 0])
    coxa_local = Point3D([coxa_length, 0, 0])
    femur_local = Point3D([femur_length, 0, 0])
    tibia_local = Point3D([tibia_length, 0, 0])

    # Create rotation matrices
    rotation_axis = np.array([0, 0, 1])
    r_alpha = R.from_rotvec(rotation_axis * alpha, degrees=True)
    r_beta = R.from_rotvec(rotation_axis * beta, degrees=True)
    r_gamma = R.from_rotvec(rotation_axis * gamma, degrees=True)

    # Calculate global positions using transformations
    body_point = start_point + body_local

    # Apply alpha rotation to coxa
    coxa_rotated = r_alpha.apply(coxa_local.numpy())
    coxa_point = body_point + coxa_rotated

    # Apply alpha+beta rotation to femur
    r_alpha_beta = r_alpha * r_beta
    femur_rotated = r_alpha_beta.apply(femur_local.numpy())
    femur_point = coxa_point + femur_rotated

    # Apply alpha+beta+gamma rotation to tibia
    r_alpha_beta_gamma = r_alpha_beta * r_gamma
    tibia_rotated = r_alpha_beta_gamma.apply(tibia_local.numpy())
    tibia_point = femur_point + tibia_rotated

    # Set  Point labels for visualization
    body_point.label = rf'$\alpha$={alpha}°'
    coxa_point.label = rf'$\beta$={beta}°'
    femur_point.label = rf'$\gamma$={gamma}°'
    tibia_point.label = 'Foot'

    # Return lines connecting the points
    return Leg3D(
        [
            Line3D(start_point, body_point, 'Body'),
            Line3D(body_point, coxa_point, 'Coxa'),
            Line3D(coxa_point, femur_point, 'Femur'),
            Line3D(femur_point, tibia_point, 'Tibia'),
        ]
    )
```

```{code-cell} ipython3
from plotting import plot_leg_with_points

coxa = 5
femur = 8
tibia = 10


model = forward_kinematics(coxa, femur, tibia, 45, -55, -14)

_ = plot_leg_with_points(
    model.xy, 'Foot on the ground (XY)', link_labels='none', x_label="X'", y_label='Y'
)
display_and_close(plt.gcf())

_ = plot_leg_with_points(
    model.xz, 'Foot on the ground (XZ)', link_labels='none', x_label="X'", y_label='Z'
)
display_and_close(plt.gcf())
```

This was a good start, but code is hard to read and understand due to excessive repetitions. Let's introduce a transform system, similar to the one used in ROS TF2 library.

Code from `geometry/transforms.py`:

```{code-cell} ipython3
:tags: [remove-cell]

import jupyter_utils

jupyter_utils.display_file('geometry/transforms.py', start_after='THE SOFTWARE.')
```

```{literalinclude} geometry/transforms.py
:start-after: THE SOFTWARE.
```

With this `AffineTransform` class we can now create a chain of transformations instead of hand crafting them. Let's rewrite the forward kinematics code using transforms.

The code below is an excerpt from `LegModel` class in `models.py` file:

```{literalinclude} models.py
:start-after: Leg Forward kinematics - START
:end-before: Leg Forward kinematics - END
```

```{code-cell} ipython3
from models import LegModel
from plotting import plot_leg3d

coxa = 5
femur = 8
tibia = 10

# model = forward_kinematics_transforms(coxa, femur, tibia, 0, -25, 110)
model = LegModel(coxa, femur, tibia)
model.forward_kinematics(0, -25, 110)

fig, _, _ = plot_leg_with_points(
    model.xy,
    'Foot in 3D (XY)',
    link_labels='none',
    joint_labels='points',
    x_label='X',
    y_label='Y',
    subplot=221,
)
_ = plot_leg_with_points(
    model.xz,
    'Foot in 3D (XZ)',
    link_labels='none',
    joint_labels='points',
    x_label='X',
    y_label='Z',
    subplot=222,
    fig=fig,
)
_ = plot_leg_with_points(
    model.yz,
    'Foot in 3D (YZ)',
    link_labels='none',
    joint_labels='points',
    x_label='Y',
    y_label='Z',
    subplot=223,
    fig=fig,
)
fig, ax, plot_data = plot_leg3d(
    model,
    'Foot in 3D',
    link_labels='none',
    joint_labels='points',
    subplot=224,
    fig=fig,
    hide_grid=False,
)
ax.plot(*zip([0, -5, 0], [0, 5, 0]), 'w:')  # add depth
ax.set_aspect('equal')  # Upset the aspect ratio
display_and_close(fig)
```

With full 3D kinematics model and plotting support lets put it all together and create a 6 legged robot model.

Code from `models.py`:

```{code-cell} ipython3
:tags: [remove-cell]

import jupyter_utils

jupyter_utils.display_file('models.py', start_after='THE SOFTWARE.')
```

```{literalinclude} models.py
:start-after: THE SOFTWARE.
```

```{code-cell} ipython3
from models import HexapodModel
from plotting import plot_hexapod

# Dr.QP Dimensions
drqp_front_offset = 0.116924  # x offset for the front and back legs
drqp_side_offset = 0.063871  # y offset fo the front and back legs
drqp_middle_offset = 0.103  # x offset for the middle legs

drqp_coxa = 0.053
drqp_femur = 0.066225
drqp_tibia = 0.123


class DrQP(HexapodModel):
    def __init__(self):
        super().__init__(
            front_offset=drqp_front_offset,
            side_offset=drqp_side_offset,
            middle_offset=drqp_middle_offset,
            coxa_len=drqp_coxa,
            femur_len=drqp_femur,
            tibia_len=drqp_tibia,
            leg_rotation=[0, 0, 45],
        )


drqp = DrQP()
drqp.forward_kinematics(0, -25, 110)
fig, ax, plot_data = plot_hexapod(drqp)
display_and_close(fig)
```

With the ability to do forward kinematics for a full robot, we can now start to work on the inverse kinematics. 1_getting_started_with_robot_ik.ipynb notebook covers the full 3D case of a single leg IK, however it works in the leg's local coordinate frame. In order to use it for the full robot, each leg global target position needs to be converted to the leg's local coordinate frame. Since we used matrix transformations for the forward kinematics, we can use the inverse of the body transform to convert the global target to the local coordinate frame.

```python
    def to_local(self, point):
        return self.body_joint.inverse().apply_point(point)
```

```{code-cell} ipython3
orig_alpha, orig_beta, orig_gamma = 0, -25, 110
drqp = DrQP()
drqp.forward_kinematics(orig_alpha, orig_beta, orig_gamma)

targets = []
for leg in drqp.legs:
    target = leg.tibia_end.copy()
    target.label = 'Target'
    target.x -= 0.08
    target.y *= 1.4

    reached = leg.move_to(target)
    if not reached:
        target.label = 'Unreachable target'
        targets.append(target)
        print(f'Leg {leg.label} failed to reach {target}, ended at {leg.tibia_end}')

fig, ax, plot_data = plot_hexapod(drqp, targets)
display_and_close(fig)
```

With the ability to position all legs, its time to work on the inverse kinematics for the body.

The algorithm is as follows:

 1. Capture the reference stance
    - Run forward kinematics for all legs with the same angles in a desired position (neutral, wide, narrow, specific gait).
    - Capture global positions of all leg foot tips
 2. Apply transform to the robot's body (translation, rotation, twist).
 3. Run inverse kinematics for all legs with the foot positions captured in step 1.

```{code-cell} ipython3
from geometry import AffineTransform

orig_alpha, orig_beta, orig_gamma = 0, -25, 110
drqp = DrQP()
drqp.forward_kinematics(orig_alpha, orig_beta, orig_gamma)

targets = [leg.tibia_end.copy() for leg in drqp.legs]

drqp.body_transform = AffineTransform.from_translation(
    [0.05, 0, -0.01]
) @ AffineTransform.from_rotvec([10, 10, 10], degrees=True)

unreachable_targets = []
for leg, target in zip(drqp.legs, targets):
    reached = leg.move_to(target)
    if not reached:
        target.label = 'Unreachable target'
        unreachable_targets.append(target)
        print(f'Leg {leg.label} failed to reach {target}, ended at {leg.tibia_end}')

fig, ax, plot_data = plot_hexapod(drqp, unreachable_targets)
display_and_close(fig)
```

```{code-cell} ipython3
import math

steps = 64
x = 0.0
y = 0.0
z = 0.0
scalar = 0.05
sequence_xy_little_circle = [
    AffineTransform.from_translation([x + math.cos(i) * scalar, y + math.sin(i) * scalar, z])
    for i in np.linspace(np.pi, np.pi * 3, steps)
]

sequence_xz_little_circle = [
    AffineTransform.from_translation([x + math.sin(i) * scalar, y, z + math.cos(i) * scalar])
    for i in np.linspace(0, np.pi * 2, steps)
]

sequence_yz_little_circle = [
    AffineTransform.from_translation([x, y + math.sin(i) * scalar, z + math.cos(i) * scalar])
    for i in np.linspace(0, np.pi * 2, steps)
]

sequence_xyz_little_circle = [
    AffineTransform.from_translation(
        [x + math.cos(i) * scalar, y + math.sin(i) * scalar, z + math.cos(i) * scalar]
    )
    for i in np.linspace(0, np.pi * 2, steps)
]
```

```{code-cell} ipython3
# Plot IK solutions and targets into an animation

from plotting import animate_plot, is_sphinx_build, update_hexapod_plot
from scipy.interpolate import interp1d

orig_alpha, orig_beta, orig_gamma = 0, -25, 110
drqp = DrQP()
drqp.forward_kinematics(orig_alpha, orig_beta, orig_gamma)

targets = [leg.tibia_end.copy() for leg in drqp.legs]

transforms = [AffineTransform.identity()]


def interpolate(tf1: AffineTransform, tf2: AffineTransform, steps=10):
    interp_func = interp1d([0, steps], np.stack([tf1.matrix, tf2.matrix]), axis=0)
    for i in range(steps):
        transforms.append(AffineTransform(interp_func(i)))


def extend_transforms(ohter_transforms, steps=10):
    interpolate(transforms[-1], ohter_transforms[0], steps=steps)
    transforms.extend(ohter_transforms)


def turn_transforms(axis, turn_degrees=10):
    axis = np.array(axis)
    for i in range(0, turn_degrees):
        transforms.append(AffineTransform.from_rotvec(axis * i, degrees=True))

    for i in range(turn_degrees, -turn_degrees, -1):
        transforms.append(AffineTransform.from_rotvec(axis * i, degrees=True))

    for i in range(-turn_degrees, 0):
        transforms.append(AffineTransform.from_rotvec(axis * i, degrees=True))


turn_transforms([0, 0, 1])
turn_transforms([0, 1, 0])
turn_transforms([1, 0, 0])
turn_transforms([1, 1, 0])
turn_transforms([1, 0, 1])
turn_transforms([1, 1, 1])

extend_transforms(sequence_xy_little_circle, steps=5)
extend_transforms(sequence_xz_little_circle, steps=5)
extend_transforms(sequence_yz_little_circle, steps=5)
extend_transforms(sequence_xyz_little_circle, steps=5)

# Close the loop
interpolate(transforms[-1], transforms[0], steps=15)

fig, ax, plot_data = plot_hexapod(drqp)


def animate(frame):
    drqp.body_transform = transforms[frame]
    for leg, target in zip(drqp.legs, targets):
        leg.move_to(target)
    update_hexapod_plot(drqp, plot_data)


save_animation = False
skip_in_local_runs_because_its_slow = not is_sphinx_build()

if not skip_in_local_runs_because_its_slow:
    animate_plot(
        fig,
        animate,
        _frames=len(transforms),
        _interval=20,
        _interactive=False,  # Interactive animation doesn't work well due to number of frames, rendering video works much better
        _save_animation_name='animation' if save_animation else None,
    )
```

To make all the learnings and findings reusable in other notebooks, lets move all the code into modules and double check it works.

```{code-cell} ipython3
from models import HexapodModel
from plotting import plot_hexapod, update_hexapod_plot

hexapod = HexapodModel()
hexapod.forward_kinematics(0, -25, 110)
fig, ax, plot_data = plot_hexapod(hexapod)

leg_tips = [leg.tibia_end.copy() for leg in hexapod.legs]

hexapod.body_transform = AffineTransform.from_translation(
    [0.05, 0, -0.01]
) @ AffineTransform.from_rotvec([10, 10, 10], degrees=True)
hexapod.move_legs_to(leg_tips)
update_hexapod_plot(hexapod, plot_data)
display_and_close(fig)
```
