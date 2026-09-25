---
type: spec
description: How the robot moves between torque-off, initialization, walking, finalization, and servo reboot, and what the brain does in each state.
generated:
  by: claude-code/opus-5
  at: 2026-09-25T12:00:00Z
sources:
- id: state
  resource: packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_machine.py
- id: node
  resource: packages/runtime/drqp_brain/drqp_brain/robot_state/robot_state_node.py
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: tests
  resource: packages/runtime/drqp_brain/test/test_robot_state_machine.py
---

# Robot lifecycle

The robot's lifecycle is owned by the `drqp_robot_state` node
(`drqp_brain/robot_state/robot_state_machine.py`). The node consumes string
events on `/robot_event` and publishes the current state name on `/robot_state`
with transient-local durability. The brain node (`brain_node.py`,
`process_robot_state`) reacts to each state change. This is a stub drafted from
the code; tighten the scenarios as tests are added.

## Requirements

### Requirement: Safe initial state

The lifecycle SHALL start in `torque_off` and publish it so that late joiners
receive the current state.

#### Scenario: Late subscriber

- **GIVEN** `drqp_robot_state` has been running in `torque_off`
- **WHEN** a new node subscribes to `/robot_state` with transient-local QoS
- **THEN** it receives `torque_off` immediately

### Requirement: Allowed transitions only

The state machine SHALL accept only these transitions and SHALL log and ignore
any other event:

- `initialize`: from `torque_off` or `finalized` to `initializing`.
- `initializing_done`: from `initializing` to `torque_on`.
- `finalize`: from `torque_on` to `finalizing`.
- `finalizing_done`: from `finalizing` to `finalized`.
- `turn_off`: from `torque_on`, `initializing`, `finalizing`, or `finalized` to
  `torque_off`.
- `reboot_servos`: from any state except `servos_rebooting` to
  `servos_rebooting`.
- `servos_rebooting_done`: from `servos_rebooting` to `torque_off`.

#### Scenario: Invalid event

- **GIVEN** the state is `torque_off`
- **WHEN** `finalize` is published on `/robot_event`
- **THEN** the state stays `torque_off` and an error is logged

### Requirement: Kill switch

The `kill_switch_pressed` event SHALL toggle the robot between rest and
activity. The joystick translator publishes it for the PS and touchpad buttons.

- From an active state (`initializing`, `torque_on`, or `finalizing`), it SHALL
  go to `torque_off`.
- From a rest state (`torque_off` or `finalized`), it SHALL go to
  `initializing`.
- In `servos_rebooting`, it SHALL be refused, and the reboot runs to completion.

`test_robot_state_machine.py` (`TestKillSwitch`) pins this table, because the
event combines `turn_off | initialize` and the library's resolution order
decides the result for `finalized`.

#### Scenario: Kill while walking

- **GIVEN** the state is `torque_on`
- **WHEN** `kill_switch_pressed` is received
- **THEN** the state becomes `torque_off`, the walk loop stops, and the brain
  publishes a zero-effort trajectory point

#### Scenario: Kill switch from rest

- **GIVEN** the state is `torque_off` or `finalized`
- **WHEN** `kill_switch_pressed` is received
- **THEN** the state becomes `initializing` and the stand-up sequence plays

#### Scenario: Kill switch during servo reboot

- **GIVEN** the state is `servos_rebooting`
- **WHEN** `kill_switch_pressed` is received
- **THEN** the state stays `servos_rebooting` and an error is logged

### Requirement: Brain reactions per state

The brain SHALL run the walking loop only in `torque_on`, and SHALL disable
balance mode whenever it leaves `torque_on`.

- `initializing`: stop the loop and play the scripted stand-up trajectory
  through the `FollowJointTrajectory` action; publish `initializing_done` when
  it completes.
- `finalizing`: stop the loop and play the scripted sit-down trajectory; publish
  `finalizing_done` when it completes.
- `torque_off` and `finalized`: publish a zero-effort trajectory point.
- `servos_rebooting`: stop the loop, send a reboot trajectory (effort −1, then
  0), and publish `servos_rebooting_done`.

#### Scenario: Stand up

- **GIVEN** the state is `torque_off`
- **WHEN** `initialize` is received
- **THEN** the robot plays the stand-up sequence (about 3.2 s), the state
  reaches `torque_on`, and the walking loop starts
