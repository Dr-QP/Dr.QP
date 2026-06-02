# Functional technical specification

## 1. Purpose

This document specifies the externally observable functionality of the Dr.QP robot MCP.

The MCP provides a tool-driven interface for:

- bringing simulation up and down,
- bringing the robot into and out of the `torque_on` lifecycle state,
- reading robot state,
- reading simulation world state,
- issuing motion commands,
- running fixed-duration walking sequences, and
- recording robot state snapshots over time.

This specification is intentionally functional rather than architectural. It describes what the MCP does, what data it returns, and how callers should expect it to behave. Internal class layout, middleware structure, process model, and other implementation details are out of scope unless they are necessary to explain behavior.

## 2. Functional scope

The Dr.QP robot MCP exposes a single logical robot control and inspection surface for the currently configured Dr.QP simulation/runtime environment.

Within scope:

- simulation bring-up and shutdown,
- robot lifecycle control for torque-on and torque-off/finalized transitions,
- current robot state inspection,
- current world state inspection,
- direct motion-command publication,
- repeated walking commands for a bounded duration,
- snapshot recording of robot state over time.

Out of scope:

- path planning,
- task planning,
- map building,
- collision checking,
- inverse kinematics APIs,
- arbitrary world mutation,
- multi-robot coordination,
- persistent storage of recordings beyond the tool response.

## 3. Operating model

The MCP behaves as a stateful control surface over a single active robot runtime.

- Calls operate against one robot instance and one active world context.
- The MCP may observe a runtime that is already active, or it may need to start the simulation environment before robot lifecycle operations can complete.
- Some information may be unavailable temporarily while the runtime is starting or while a data source has not yet produced messages.
- World-state inspection is intended for Gazebo-backed simulation environments.

## 4. Core functional concepts

### 4.1 Robot lifecycle state

The MCP reports and reacts to a robot lifecycle state. Observed states include:

- `torque_on`
- `torque_off`
- `initializing`
- `finalizing`
- `finalized`
- `servos_rebooting`
- unavailable or unknown state represented as `null`

These states are used by lifecycle tools to determine whether work is required, whether a state transition must be awaited, or whether the request cannot be completed.

### 4.2 Robot state snapshot

A robot state snapshot is the main read model for the robot. It represents the latest known robot condition and includes:

- a timestamp for when the snapshot was produced,
- whether any robot-related data is currently available,
- whether simulation appears to be running,
- the current lifecycle state if known,
- the current world name when simulation is active,
- current simulation time if known,
- the robot pose,
- joint states keyed by joint name,
- an optional note when data is unavailable or incomplete.

The robot pose is functionally defined as the robot pose exposed by odometry. This matters because callers can rely on the pose matching the live robot pose stream rather than a separate world-query approximation.

### 4.3 World state snapshot

A world state snapshot represents the latest known world entity poses for the active simulation world and includes:

- whether world-state data is available,
- the world name,
- simulation time if known,
- entity count,
- a list of entities,
- the source identifier,
- an optional note when world data is unavailable or partially unavailable.

Each world entity contains:

- entity name,
- entity identifier when available,
- entity pose.

World state is functionally defined as coming from the live Gazebo world pose stream when available.

### 4.4 Motion command

A motion command is a normalized joystick-like command that combines:

- stride direction in `x`, `y`, and `z`,
- rotational speed,
- body translation in `x`, `y`, and `z`,
- body rotation as roll, pitch, and yaw,
- gait type.

All numeric motion inputs are normalized values in the inclusive range `[-1.0, 1.0]`.

Supported gait types are:

- `tripod`
- `ripple`
- `wave`

Gait names are treated case-insensitively and normalized before use.

### 4.5 Recording session

A recording session captures a sequence of robot state snapshots at a fixed sampling interval.

A recording session has:

- active or idle state,
- sample interval,
- start timestamp,
- current sample count while active,
- final sample list when stopped.

Only one recording session may be active at a time.

## 5. Exposed MCP tools

The MCP exposes the following tools.

### 5.1 `drqp_robot_sim_bring_up`

Purpose:

- make the simulation environment available without also asserting a robot lifecycle transition.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Attempts to ensure the simulation environment is running.
- If simulation is already running, returns success without changing robot lifecycle state.
- If simulation must be started, waits for simulation-related state to become observable.
- Does not require the robot to reach `torque_on` as part of the contract.
- May return robot state information when it becomes available during simulation startup.
- Fails when simulation cannot be started or does not become observable within the timeout.

Output:

- lifecycle or environment action result containing:
	action,
	state before,
	state after,
	whether simulation was started by this call,
	whether simulation is running at completion,
	a user-facing message,
	an optional log path.

Functional guarantees:

- successful completion means simulation is available for follow-on robot inspection or control operations;
- repeated calls are safe when simulation is already running.

### 5.2 `drqp_robot_torque_on`

Purpose:

- bring the robot to the `torque_on` lifecycle state.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Reads current robot state before acting.
- Assumes simulation may already be running or may have been brought up separately.
- If no lifecycle information is available but robot state becomes observable, treats the robot as bootable from an off state.
- If the robot is already in `torque_on`, returns success without changing state.
- If the robot is in `torque_off` or `finalized`, requests initialization and waits for `torque_on`.
- If the robot is already `initializing`, waits for `torque_on`.
- If the robot is `finalizing`, waits for `finalized`, then requests initialization and waits for `torque_on`.
- If the robot is `servos_rebooting`, waits for `torque_off`, then requests initialization and waits for `torque_on`.
- Fails if lifecycle state remains unavailable and the MCP cannot complete the sequence.

Output:

- lifecycle action result containing:
	action,
	state before,
	state after,
	whether simulation was started by this call,
	whether simulation is running at completion,
	a user-facing message,
	an optional log path.

Functional guarantees:

- successful completion means the robot reached `torque_on`;
- repeated calls are safe when the robot is already in `torque_on`.

### 5.3 `drqp_robot_torque_off`

Purpose:

- move the robot out of the active `torque_on` state without requiring simulation shutdown.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Reads current robot state before acting.
- If lifecycle state is unavailable, returns a structured result rather than forcing a transition.
- If current state is `torque_on`, requests finalization and waits for `finalized`.
- If current state is `initializing`, requests turn-off and waits for `torque_off`.
- If current state is `finalizing`, waits for `finalized`.
- If current state is already `finalized` or `torque_off`, returns that state as the post-condition.
- If current state is `servos_rebooting`, waits for `torque_off`.
- Fails on unsupported lifecycle states.
- Does not require simulation to stop as part of the contract.

Output:

- lifecycle action result with the same shape as torque-on.

Functional guarantees:

- successful completion means the robot is no longer in the active `torque_on` state;
- repeated calls are safe when the robot is already not active.

### 5.4 `drqp_robot_sim_shut_down`

Purpose:

- stop the simulation environment after robot activity has been brought down or is otherwise safe to stop.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Attempts to stop the active simulation environment.
- If simulation is already stopped or unavailable, returns a structured success or no-op result.
- Does not imply any robot lifecycle transition on its own; callers should use `drqp_robot_torque_off` first when they need orderly robot shutdown before simulation stop.
- Waits for simulation to become unavailable or non-running when the environment supports an orderly shutdown contract.

Output:

- lifecycle or environment action result with the same general shape as simulation bring-up.

Functional guarantees:

- successful completion means simulation is no longer available to the MCP or is confirmed already stopped.

### 5.5 `drqp_robot_get_state`

Purpose:

- return the latest robot lifecycle, pose, time, and joint-state snapshot.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for any robot-related runtime data to arrive.
- Returns the latest cached state even if only partial data is available.
- Marks the snapshot as unavailable only when no lifecycle, pose, joint, or simulation-time data is present.
- Marks simulation as running when simulation time is available.

Output:

- robot state snapshot.

Functional semantics:

- `available = true` means at least one meaningful robot-related data source is available;
- `simulation_running = true` means simulation time is advancing or at least present;
- `note` explains missing data when the snapshot is empty.

### 5.6 `drqp_robot_start_state_recording`

Purpose:

- begin sampling robot state at a fixed interval.

Input:

- `sample_interval_sec` with default `0.5`.

Behavior:

- Starts a new recording session immediately.
- Captures robot state snapshots repeatedly until stopped.
- Rejects non-positive sampling intervals.
- Rejects attempts to start a second recording while one is already active.

Output:

- recording status marked active with:
	current interval,
	zero initial sample count,
	start timestamp,
	status message.

### 5.7 `drqp_robot_stop_state_recording`

Purpose:

- stop the active recording session and return the captured samples.

Behavior:

- Stops the active recording loop.
- Waits for the recording worker to finish.
- Returns all samples captured so far.
- Rejects requests when no recording session is active.

Output:

- recorded robot states containing:
	start timestamp,
	stop timestamp,
	sampling interval,
	sample count,
	ordered list of robot state snapshots.

Functional guarantees:

- samples are returned in capture order;
- `sample_count` equals the length of `samples`.

### 5.8 `drqp_robot_get_recording_status`

Purpose:

- report whether recording is active.

Behavior:

- Returns an active status when recording is underway.
- Returns an idle status when no session exists.

Output:

- recording status containing:
	active flag,
	sample interval or `null`,
	current sample count,
	start timestamp or `null`,
	status message.

### 5.9 `drqp_robot_get_world_state`

Purpose:

- return the latest known simulated world entity poses.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Attempts to obtain the active world pose stream.
- Waits briefly for a world-state update.
- Returns the latest cached world state when available.
- If no world-state message has arrived, still reports simulation time when available.
- Returns a structured unavailable or partial result rather than failing when world-state transport is absent.

Output:

- world state snapshot.

Functional semantics:

- `available = true` can mean either entity data is available or simulation is known to be running;
- `entity_count` may be zero when the world stream is unavailable or has not produced data yet;
- `note` explains transport or availability issues.

### 5.10 `drqp_robot_send_motion_command`

Purpose:

- publish a single normalized motion command for walking or body motion.

Inputs:

- `stride_x`, `stride_y`, `stride_z`,
- `rotation_speed`,
- `body_x`, `body_y`, `body_z`,
- `body_roll`, `body_pitch`, `body_yaw`,
- `gait_type`.

Defaults:

- all numeric inputs default to `0.0`;
- gait defaults to `tripod`.

Behavior:

- Validates every numeric input against the inclusive range `[-1.0, 1.0]`.
- Validates gait type against the supported gait set.
- Publishes the requested command.
- Returns the normalized command content that was accepted.

Output:

- motion command result containing:
	stride direction,
	rotation speed,
	body translation,
	body rotation,
	gait type,
	status message.

Functional guarantees:

- the response reflects the effective command values after normalization of gait casing and whitespace;
- invalid input is rejected before publication.

### 5.11 `drqp_robot_stop_motion`

Purpose:

- stop walking and body motion by publishing a zeroed motion command.

Behavior:

- Equivalent to sending a motion command with all axes set to zero and gait `tripod`.

Output:

- motion command result for the zeroed command.

### 5.12 `drqp_robot_walk_for_duration`

Purpose:

- repeat the same motion command for a fixed duration, optionally followed by an explicit stop command.

Inputs:

- `duration_sec`,
- `publish_hz` with default `5.0`,
- `stop_after` with default `true`,
- the same motion parameters accepted by `drqp_robot_send_motion_command`.

Behavior:

- Rejects non-positive `duration_sec`.
- Rejects non-positive `publish_hz`.
- Computes `publish_count` as the ceiling of `duration_sec * publish_hz`, with a minimum of one publish.
- Publishes the requested motion command exactly `publish_count` times.
- Waits approximately `1 / publish_hz` seconds between publishes except after the final publish.
- If `stop_after` is true, sends a zeroed stop command after the repeated command sequence.

Output:

- motion sequence result containing:
	requested duration,
	publish rate,
	publish count,
	whether a stop command was sent,
	the final non-zero command result,
	status message.

Functional guarantees:

- the repeated sequence is bounded in time by the requested duration and publish rate contract;
- when `stop_after = true`, a stop command is issued after the final movement publish.

## 6. Response models

### 6.1 Pose model

A pose contains:

- `position` with `x`, `y`, `z`
- `orientation` with quaternion `x`, `y`, `z`, `w`

### 6.2 Joint-state model

Joint states are returned as a mapping from joint name to:

- `position`
- `velocity`
- `effort`

Each field may be `null` when that value is not present for a joint.

### 6.3 Lifecycle action result

Lifecycle action responses contain:

- `action`
- `state_before`
- `state_after`
- `simulation_was_started`
- `simulation_running`
- `message`
- `log_path`

### 6.4 Motion command result

Motion command responses contain:

- normalized stride direction vector,
- normalized rotation speed,
- normalized body translation vector,
- normalized body rotation vector,
- gait type,
- message.

### 6.5 Motion sequence result

Walking sequence responses contain:

- `duration_sec`
- `publish_hz`
- `publish_count`
- `stop_command_sent`
- `command`
- `message`

### 6.6 Recording status

Recording status responses contain:

- `active`
- `sample_interval_sec`
- `sample_count`
- `started_at`
- `message`

### 6.7 Recorded robot states

Completed recording responses contain:

- `started_at`
- `stopped_at`
- `sample_interval_sec`
- `sample_count`
- `samples`

## 7. Validation and error behavior

### 7.1 Input validation

The MCP rejects requests in these cases:

- any motion axis or rotation input outside `[-1.0, 1.0]`,
- unsupported gait type,
- non-positive recording interval,
- non-positive walk duration,
- non-positive walk publish rate,
- attempt to start a recording when one is already active,
- attempt to stop recording when no recording is active.

### 7.2 Timeout behavior

Timeouts are used to bound waits for:

- robot state availability,
- lifecycle state transitions,
- world-state availability.

On timeout:

- state-reading tools return the best currently known partial snapshot when possible;
- lifecycle-transition tools fail when the requested terminal state is not reached in time;
- simulation bring-up or shutdown tools fail when simulation availability does not reach the requested condition in time.

### 7.3 Partial availability

The MCP favors structured partial responses over hard failure for inspection operations.

Examples:

- robot state may be returned with `available = false` and a note when no robot topics have appeared yet;
- world state may be returned with simulation time but zero entities when Gazebo world-pose data is not yet available;
- shutdown may return a structured result when lifecycle state is unavailable rather than forcing an invalid transition.

### 7.4 Publication semantics

For lifecycle and motion commands, successful completion means the MCP accepted the request and published the corresponding command or event. It does not by itself guarantee that downstream robot behavior has physically completed, except where the tool explicitly waits for a lifecycle state transition.

`drqp_robot_walk_for_duration` is higher level: it guarantees repeated command publication according to the request and, when enabled, an explicit stop command afterward.

## 8. Observable behavior requirements

The current MCP behavior establishes the following functional requirements.

### 8.1 Simulation bring-up and shutdown

- The MCP shall expose a simulation bring-up operation distinct from robot lifecycle activation.
- The MCP shall attempt to make simulation available when the environment supports simulation startup.
- The MCP shall return an idempotent success or no-op result when simulation is already running.
- The MCP shall expose a simulation shutdown operation distinct from robot lifecycle deactivation.

### 8.2 Torque-on and torque-off behavior

- The MCP shall be able to bring the robot to `torque_on` from an already available `torque_off` state.
- The MCP shall treat an available robot state with no lifecycle state but with live simulation or joint data as effectively bootable from an off state.
- The MCP shall return an idempotent success result when the robot is already in `torque_on`.
- The MCP shall be able to shut the robot down from `torque_on` without implying simulation shutdown.
- The MCP shall tolerate torque-off requests when the robot is already shut down.
- The MCP shall not invent lifecycle state when none is observable.

### 8.3 State inspection

- The MCP shall expose the latest robot pose from odometry.
- The MCP shall expose simulation time when available.
- The MCP shall expose joint states keyed by joint name.
- The MCP shall keep returning the latest known snapshot rather than requiring a new state message for every request.

### 8.4 World inspection

- The MCP shall expose world entities with names, identifiers, and poses when Gazebo world-state data is available.
- The MCP shall identify the world-state source as Gazebo.
- The MCP shall continue to report advancing simulation time across successive snapshots when the simulation is running.

### 8.5 Motion control

- The MCP shall allow callers to command stride, rotation, and body pose offsets through normalized inputs.
- The MCP shall reject invalid movement magnitudes before sending a command.
- The MCP shall expose a dedicated stop command.
- The MCP shall support at least the `tripod`, `ripple`, and `wave` gait modes.

### 8.6 Recording

- The MCP shall capture multiple robot-state samples during an active recording session.
- The MCP shall report live sample count while recording is active.
- The MCP shall return the full captured sample list when recording stops.

## 9. Assumptions visible to callers

The following assumptions are functionally relevant to callers:

- The MCP operates against the current ROS 2 environment.
- Robot pose is sourced from odometry.
- World state is sourced from Gazebo world pose information when Gazebo is available.
- The returned world name identifies the active simulation world known to the MCP.
- Tool responses are snapshots of the latest known state, not a continuously streaming session.

## 10. Non-functional exclusions

This specification does not define:

- internal concurrency model,
- topic names except where required to explain behavior contracts,
- transport libraries,
- server transport mode,
- dependency-loading strategy,
- process-supervision approach,
- file layout,
- test strategy,
- implementation ownership boundaries.

Those concerns may change without changing the functional contract described here.
