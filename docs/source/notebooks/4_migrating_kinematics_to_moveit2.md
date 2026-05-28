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

# 4. Migrating the Existing Kinematics Model to MoveIt 2

This notebook explains how the original Dr.QP kinematics model is translated into a MoveIt 2 setup.

The important idea is that the old model is not thrown away. Instead, its assumptions are re-expressed in the formats MoveIt understands:

- robot geometry becomes URDF/Xacro
- planning semantics become SRDF
- inverse kinematics becomes a MoveIt solver configuration
- trajectory execution is connected to `ros2_control`
- launch files assemble the full system

The goal of this notebook is to make that translation easy to follow, especially if you are new to ROS, URDF, or MoveIt.

## What existed before MoveIt 2?

Before MoveIt was added, Dr.QP already had a useful analytical model in `drqp_brain`.

At a high level:

- `HexapodModel` creates six legs with fixed mounting offsets and yaw rotations.
- each `LegModel` knows its three link lengths: coxa, femur, and tibia
- `forward_kinematics()` computes the foot position from joint angles
- `inverse_kinematics()` computes joint angles for a target foot position

That analytical model is excellent for understanding the robot and for building gait logic. MoveIt 2, however, expects a robot description and a planning configuration rather than a handwritten Python solver.

## The old mental model in one picture

The original Python model is based on a simple idea: every leg is the same 3-DOF chain, but each leg is mounted at a different pose on the body.

```{note}
This is the key observation that makes the migration manageable.

If the six legs share the same internal structure, then the MoveIt 2 model can reuse one leg description many times with different prefixes and mount poses.
```

In `HexapodModel`, the six legs are created by changing only two kinds of information:

- where the leg is attached to the body
- how the leg is rotated around the body

The actual leg math remains the same for every leg.

## Step 1. Preserve the robot's structure in URDF/Xacro

MoveIt does not read Python classes like `HexapodModel`. It reads a robot description, usually generated from URDF and Xacro.

That means the first migration step is to turn the same physical assumptions into links and joints.

### 1.1 The body file instantiates six legs

The top-level robot description lives in `drqp.urdf.xacro`. It mounts the same leg macro six times with different positions and yaw values.

```xml
<xacro:leg robot_name="$(arg robot_name)" prefix="left_front" x="0.11692" y="0.06387" yaw="${pi/4}"/>
<xacro:leg robot_name="$(arg robot_name)" prefix="right_front" x="0.11692" y="-0.06387" yaw="${-pi/4}"/>

<xacro:leg robot_name="$(arg robot_name)" prefix="left_middle" x="0" y="0.103" yaw="${pi/2}"/>
<xacro:leg robot_name="$(arg robot_name)" prefix="right_middle" x="0" y="-0.103" yaw="${-pi/2}"/>

<xacro:leg robot_name="$(arg robot_name)" prefix="left_back" x="-0.11692" y="0.06387" yaw="${3*pi/4}"/>
<xacro:leg robot_name="$(arg robot_name)" prefix="right_back" x="-0.11692" y="-0.06387" yaw="${-3*pi/4}"/>
```

This is the URDF/Xacro equivalent of the `location_on_body` and `rotation` fields used by `LegModel` in Python.

### 1.2 The leg macro defines the same three actuated joints

Inside `leg.urdf.xacro`, each leg contains:

- a `coxa` revolute joint around the Z axis
- a `femur` revolute joint around the Y axis
- a `tibia` revolute joint around the Y axis

Those axes match the old analytical model:

- alpha -> coxa yaw
- beta -> femur pitch
- gamma -> tibia pitch

The URDF also stores joint limits directly on the joints. For example:

```xml
<joint name="${robot_name}/${prefix}_coxa" type="revolute">
  <axis xyz="0 0 1"/>
  <limit effort="3" lower="${radians(-90)}" upper="${radians(90)}" velocity="${pi}"/>
</joint>

<joint name="${robot_name}/${prefix}_femur" type="revolute">
  <axis xyz="0 1 0"/>
  <limit effort="3" lower="${radians(-98)}" upper="${radians(90)}" velocity="${pi}"/>
</joint>

<joint name="${robot_name}/${prefix}_tibia" type="revolute">
  <axis xyz="0 1 0"/>
  <limit effort="3" lower="${radians(-80)}" upper="${radians(110)}" velocity="${pi}"/>
</joint>
```

```{important}
URDF does not store "how to solve IK". It only stores the robot's structure: links, joints, axes, origins, and limits.
```

### 1.3 Why there are many fixed joints inside one leg

If you are new to URDF, the leg file may look more complicated than the Python model because it includes servo bodies, brackets, and several fixed joints.

That is normal.

The analytical model collapses the hardware into three clean segments. The URDF keeps the real mechanical parts because visualization, collision checking, and simulation need the actual physical arrangement.

The important thing is that the three actuated joints are still the same three logical degrees of freedom.

## Step 2. Add the semantics MoveIt needs through SRDF

URDF tells MoveIt what the robot is.

SRDF tells MoveIt how to reason about the robot.

The SRDF for Dr.QP lives in `packages/runtime/drqp_moveit/config/drqp.srdf` and adds four important layers of meaning.

### 2.1 Planning groups

Each leg becomes its own MoveIt planning group, and all six groups are combined into a `whole_body` group.

Conceptually, this is the MoveIt version of saying:

- "solve one leg on its own" or
- "plan for all 18 joints together"

Example:

```xml
<group name="left_front_leg">
  <chain base_link="drqp/base_center_link"
         tip_link="drqp/left_front_foot_link"/>
</group>

<group name="whole_body">
  <group name="left_front_leg"/>
  <group name="right_front_leg"/>
  <group name="left_middle_leg"/>
  <group name="right_middle_leg"/>
  <group name="left_back_leg"/>
  <group name="right_back_leg"/>
</group>
```

### 2.2 End effectors

Each foot is declared as an end effector. This gives MoveIt a clean "tip" to target when solving motion for a leg.

### 2.3 Named states

The SRDF defines a `home` state for every leg and for the `whole_body` group. That gives the planner a known neutral configuration.

### 2.4 Disabled collision pairs

MoveIt checks for self-collisions during planning. For a hexapod, some link pairs are always adjacent or can never physically collide.

Those pairs are listed explicitly in the SRDF so MoveIt does not waste time checking impossible collisions.

```{tip}
This part is easy to underestimate.

If your groups are correct but your self-collision matrix is poor, planning can look broken even though the kinematics are fine.
```

## Step 3. Replace the handwritten IK entry point with a MoveIt solver plugin

In the analytical model, IK is solved by Python code inside `LegModel.inverse_kinematics()`.

In MoveIt 2, IK is selected per planning group in `kinematics.yaml`.

For Dr.QP, each leg is a standard serial chain, so the migration uses MoveIt's KDL plugin instead of writing a custom solver plugin.

```{literalinclude} ../../../packages/runtime/drqp_moveit/config/kinematics.yaml
:language: yaml
```

This is one of the most important migration choices:

- the old system encoded the solver algorithm in Python
- the new system encodes the solver choice in configuration

That is possible because the URDF/SRDF now describe the kinematic chain well enough for a generic plugin to solve it.

## Step 4. Carry over limits and execution constraints

The old analytical solver knew what kinds of poses were possible because of geometry and angle calculations.

MoveIt still needs explicit planning constraints, especially for trajectory generation and execution.

### 4.1 Joint limits

The per-joint planning limits live in `joint_limits.yaml`. These values work alongside the limits already declared in URDF.

The file covers all 18 joints so MoveIt can apply velocity constraints consistently during planning.

### 4.2 Controller mapping

MoveIt plans in terms of joint trajectories, but it still needs to know which ROS action server will execute those trajectories.

That bridge is defined in `moveit_controllers.yaml`.

```{literalinclude} ../../../packages/runtime/drqp_moveit/config/moveit_controllers.yaml
:language: yaml
```

This is an important architectural point:

- MoveIt is not replacing `ros2_control`
- MoveIt is producing trajectories for the controller that already exists

So the migration is not just "make IK work in RViz". It is also "connect the planner to the robot's current execution path".

## Step 5. Assemble everything in the launch files

Once the URDF, SRDF, solver config, limits, and controller mapping exist, the system still has to be launched with the correct parameters.

That is the job of the launch files in `drqp_moveit/launch/`.

The key file for understanding the integration is `demo.launch.py`.

Its job is to load:

- `robot_description` from Xacro
- `robot_description_semantic` from SRDF
- `robot_description_kinematics` from `kinematics.yaml`
- `robot_description_planning` from `joint_limits.yaml`
- OMPL planning settings
- controller configuration

Then it starts:

- `robot_state_publisher`
- `joint_state_publisher` or `joint_state_publisher_gui`
- `move_group`
- `rviz2`

In other words, the launch file is the place where the migration becomes a running system rather than a collection of files.

## Step 6. Support both visualization-only and simulation flows

Dr.QP keeps two beginner-friendly entry points.

### 6.1 RViz-only demo

Use this when you want to inspect the robot, move joints manually, and experiment with planning without hardware or Gazebo.

```bash
ros2 launch drqp_moveit demo.launch.py
```

### 6.2 Gazebo demo

Use this when you want MoveIt connected to the simulated robot stack.

```bash
ros2 launch drqp_moveit demo_gazebo.launch.py
```

The Gazebo launch reuses the same MoveIt parameters, but runs with simulated time and includes the simulation stack.

This is a strong sign that the migration is properly layered:

- one robot description
- one MoveIt configuration package
- multiple runtime entry points

## Step 7. Verify the migration with automated tests

The migration is also protected by tests in `test_moveit_config_package.py`.

Those tests check that:

- the package manifest declares the required dependencies
- the YAML files parse and expose the expected groups
- the SRDF declares all six legs plus `whole_body`
- the launch files create the expected MoveIt and RViz nodes
- the runtime parameters include robot description, semantics, kinematics, and planning limits

This matters because MoveIt migrations often fail in configuration, not in code.

For example, a typo in a group name can silently break planning even though the URDF is valid XML. Tests catch that kind of failure early.

## Old world vs new world

The table below is the shortest way to remember the migration.

| Old analytical concept             | MoveIt 2 representation                                       |
| ---------------------------------- | ------------------------------------------------------------- |
| `HexapodModel` leg placement       | leg macro instances in `drqp.urdf.xacro`                      |
| `LegModel` 3-DOF chain             | revolute joints in `leg.urdf.xacro`                           |
| handwritten `inverse_kinematics()` | KDL plugin in `kinematics.yaml`                               |
| foot target for a leg              | SRDF chain group with foot tip link                           |
| neutral/default pose               | SRDF `group_state` named `home`                               |
| direct angle output                | planned trajectory sent through `joint_trajectory_controller` |
| ad hoc runtime assembly            | launch files that populate `move_group` parameters            |

## What did not change

It is easy to think of this migration as "replacing the old kinematics with MoveIt". That is only partly true.

These things remain the same:

- the robot still has six identical 3-DOF legs
- the important joint axes are still one yaw joint and two pitch joints per leg
- the same joint limits still define what is physically legal
- higher-level behavior such as gait generation still depends on the same robot structure

What changed is the interface around the model:

- the robot is now described declaratively
- IK is selected through plugin configuration
- planning and collision checking are handled by MoveIt
- execution is routed through a standard ROS control path

## A practical migration checklist

If you want to repeat this process for another robot, use this checklist:

1. identify the true actuated joints and their axes
2. encode the physical structure in URDF or Xacro
3. define planning groups and end effectors in SRDF
4. pick a solver plugin for each planning group
5. declare planning limits and controller mappings
6. launch `move_group` with all required robot description parameters
7. add tests for group names, parameter loading, and launch wiring

## Summary

The Dr.QP migration to MoveIt 2 is best understood as a translation exercise.

The original Python kinematics model supplied the robot knowledge:

- how legs are mounted
- how joints move
- where the feet are
- which poses are reachable

The MoveIt 2 system takes the same knowledge and spreads it across the files and runtime components that ROS tooling expects.

Once that translation is complete, Dr.QP gains standard planning, collision checking, RViz integration, and controller execution without losing the structure that the earlier notebooks developed step by step.
