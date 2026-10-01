---
type: spec
description: How semantic MovementCommand input becomes gait motion and joint trajectories in the brain's control loop.
generated:
  by: claude-code/opus-5
  at: 2026-09-25T00:00:00Z
sources:
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: msg
  resource: packages/runtime/drqp_interfaces/msg/MovementCommand.msg
---

# Locomotion command

How `drqp_brain` (`brain_node.py`) turns `/robot/movement_command`
(`drqp_interfaces/MovementCommand`) into walking. This is a stub drafted from
the code. A metric `/cmd_vel` input (roadmap RM-02) is not specified here yet.

## Requirements

### Requirement: Semantic command semantics

A `MovementCommand` SHALL be interpreted as normalized intent, not metric
values:

- `stride_direction` (x forward, y left, z up) and `rotation_speed` (positive
  means counter-clockwise) are in the range −1…1.
- `body_translation` and `body_rotation` offset the body from the neutral
  stance, also normalized.
- `gait_type` is one of `tripod`, `ripple`, or `wave`. An unknown name keeps the
  current gait.

The latest command SHALL persist until the next one arrives. Commands SHALL be
ignored while balance mode is enabled.

#### Scenario: Gait switch

- **GIVEN** the robot is walking in `tripod`
- **WHEN** a command arrives with `gait_type = "wave"`
- **THEN** subsequent ticks use the wave gait with a 2.5 s cycle time (tripod
  1.25 s, ripple 1.5625 s)

### Requirement: Control loop rate

The walking loop SHALL tick at the `control_rate_hz` parameter, which defaults
to 25 Hz. Values outside `[CONTROL_RATE_MIN_HZ, CONTROL_RATE_MAX_HZ]` SHALL be
rejected at startup. Gait timing SHALL be time-based, so the walking speed does
not depend on the tick rate.

#### Scenario: Different rate, same speed

- **GIVEN** the same command at `control_rate_hz` 12.5 and at 25
- **WHEN** the robot walks for the same duration
- **THEN** the gait cycles advance by the same amount of time

### Requirement: Trajectory publication

Each tick SHALL publish one `JointTrajectory` on
`/joint_trajectory_controller/joint_trajectory` with points spaced
`1 / control_rate_hz` apart. The loop SHALL skip publishing when the motion
state is unchanged. It SHALL skip a tick, with a logged warning or error, when
kinematics rejects the foot targets or IK is not ready.

#### Scenario: Standing still

- **GIVEN** `torque_on` and an all-zero command
- **WHEN** the stance has already been published
- **THEN** no further trajectories are published until the command changes

#### Scenario: Unreachable targets

- **WHEN** a command drives a foot target outside the reachable workspace
- **THEN** that tick publishes nothing and a warning is logged, and the
  controller holds the last trajectory

### Requirement: Kinematics backend

IK SHALL use the `analytic` backend by default; `kinematics_backend = moveit`
selects MoveItPy. Any other value SHALL fail at startup.
