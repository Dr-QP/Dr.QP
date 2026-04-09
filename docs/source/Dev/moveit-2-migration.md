# MoveIt 2 IK migration

Dr.QP already has most of the ingredients needed for a MoveIt 2-based
inverse kinematics stack:

- the current leg math and solver behaviour documented in the notebooks,
- a runtime implementation in `packages/runtime/drqp_brain/drqp_brain/models.py`,
- and a ROS 2 robot description plus `ros2_control` integration in
  `packages/runtime/drqp_control`.

This page turns the backlog item “Migrate IK to MoveIt 2” into a concrete
implementation plan for the current Jazzy-based workspace.

## Recommended target architecture

1. Keep the current `LegModel.inverse_kinematics()` behaviour as the
   reference contract for a single leg.
2. Use `packages/runtime/drqp_control/urdf/dr_qp.urdf.xacro` as the source
   of truth for `robot_description`.
3. Generate a MoveIt 2 configuration package with planning groups for each
   leg before attempting full-body planning.
4. Reuse the existing `joint_trajectory_controller` from
   `packages/runtime/drqp_control/launch/ros2_controller.launch.py` for
   trajectory execution.
5. Validate everything in RViz2 and simulation before touching hardware.

## Migration plan

### 1. Freeze the current IK contract

Treat the existing solver as executable documentation before moving it
behind ROS 2 or MoveIt 2 APIs.

- Keep the notebooks under `docs/source/notebooks/` as the mathematical
  explanation of the solver.
- Preserve and extend the parity checks in
  `packages/runtime/drqp_brain/test/test_solver.py`.
- Document the expected joint order, limits, and coordinate frames for each
  leg so the MoveIt setup matches the current solver assumptions.

### 2. Build a MoveIt-ready robot description

MoveIt 2 needs a clean robot model before it can plan anything useful.

- Start from `packages/runtime/drqp_control/urdf/dr_qp.urdf.xacro`.
- Generate an SRDF with the MoveIt Setup Assistant.
- Create planning groups per leg first; whole-body planning can come later.
- Generate and review the self-collision matrix, especially around the body,
  neighbouring femurs, and feet.

### 3. Port the solver at the boundary, not in the middle

The trigonometry should remain testable Python or library code. The ROS 2
and MoveIt 2 integration layer should stay thin.

- Wrap the solver as a MoveIt 2 kinematics plugin or a planning adapter.
- Keep any Mithi-derived leg math isolated from ROS-specific concerns.
- Expose target poses, stances, or gait goals through ROS 2 interfaces
  rather than calling notebook code directly.

### 4. Integrate trajectory execution with ROS 2 Control

MoveIt 2 will plan trajectories, but `ros2_control` still owns execution.

- Reuse the existing `joint_trajectory_controller`.
- Add the matching MoveIt controller configuration instead of introducing a
  parallel execution path.
- Account for ROS 2 parameters, remapping, and lifecycle behaviour when
  launching the planning stack.

### 5. Validate in simulation first

Simulation is the safest place to shake out interface mismatches.

- Visualize plans in RViz2.
- Execute trajectories in the existing simulator/Gazebo path.
- Compare planned joint values against the current solver for representative
  reachable targets.
- Confirm unreachable targets fail predictably and do not produce unsafe
  commands.

## Suggested acceptance criteria

- A MoveIt 2 config package loads Dr.QP successfully in RViz2.
- At least one leg planning group can solve and plan to target poses with
  parity against the current IK solver.
- Planned trajectories execute through the existing
  `joint_trajectory_controller`.
- Simulation demonstrates collision-aware planning and stable handling of
  unreachable targets.

## References

- [MoveIt 2 documentation](https://moveit.picknik.ai/)
- [MoveIt 2 tutorials](https://moveit.picknik.ai/main/doc/tutorials/tutorials.html)
- [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
