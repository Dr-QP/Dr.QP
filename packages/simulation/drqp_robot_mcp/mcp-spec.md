# Functional technical specification

## 1. Purpose

This document specifies the externally observable functionality of the Dr.QP robot MCP.

The specification defines a namespace-based MCP interface with a clean separation between:

- simulation control and simulated state,
- robot lifecycle and motion control,
- system-level ROS runtime and transport state.

The MCP must support both request-response tools and streaming state interfaces. The request-response tools are used for imperative actions and point-in-time inspection. The streaming interfaces are used for continuous state observation.

This is a functional specification. It defines capabilities, interface shape, returned data, expected behavior, and observable guarantees. It does not prescribe implementation structure, transport library choice, topic wiring, process layout, or concurrency design except where required to define behavior.

## 2. Design goals for the interface

The MCP interface shall:

- present a clear namespace hierarchy,
- separate simulation state from robot state,
- separate controllable robot actions from passive system observation,
- provide point-in-time state reads,
- provide streaming state interfaces for all major state surfaces,
- allow clients to compose simulation control, robot control, and system inspection independently.

## 3. Top-level namespaces

The MCP interface is organized into three namespaces.

### 3.1 `simulation`

The `simulation` namespace is responsible for simulation lifecycle and simulated state.

This namespace owns:

- starting simulation,
- stopping simulation,
- reading simulation runtime state,
- reading simulated robot state,
- reading simulated world state,
- streaming simulation state,
- streaming simulated robot state,
- streaming simulated world state.

### 3.2 `robot`

The `robot` namespace is responsible for robot lifecycle, robot-local state, motion control, and robot-state recording.

This namespace owns:

- booting the robot into an active lifecycle state,
- shutting the robot down through its lifecycle,
- reading robot lifecycle and joint state,
- issuing motion commands,
- stopping motion,
- running bounded walking sequences,
- managing robot-state recording,
- streaming robot state.

### 3.3 `system`

The `system` namespace is responsible for ROS and MCP runtime state that does not belong exclusively to either simulation state or robot state.

This namespace owns:

- reading system runtime state,
- streaming system runtime state,
- exposing the availability and health of the data sources required by the `robot` namespace and, when present, the `simulation` namespace.

## 4. Namespace boundaries

### 4.1 Simulation state

Simulation state answers questions such as:

- Is the simulation running?
- Is simulation time available?
- What world is active?
- What are the current entity poses in the world?
- What is the simulated pose of the robot in the world?

Simulation state is world-oriented and environment-oriented.

### 4.2 Robot state

Robot state answers questions such as:

- What lifecycle state is the robot in?
- Are joint states available?
- What are the current joint values?
- Is the robot active for torque-enabled behavior?
- What robot command state was most recently accepted?

Robot state is robot-oriented and control-oriented.

### 4.3 System state

System state answers questions such as:

- Is the ROS runtime available?
- Is the MCP runtime connected to the required data sources?
- Are robot state and motion command channels available?
- Are simulation channels available?
- Are there degraded or unavailable subsystems?

System state is infrastructure-oriented and integration-oriented.

## 5. Operating model

The MCP behaves as a stateful control surface over one active Dr.QP runtime context.

- Calls operate against one logical robot and, when simulation exists, one active world context.
- The MCP may run against a simulated robot or a real robot.
- Simulation may be absent entirely when the MCP is attached to a real robot runtime.
- Simulation may be already running before the MCP is used.
- Robot lifecycle state may be unavailable until the relevant runtime data appears.
- State tools return the latest known consistent snapshot for their namespace.
- Streaming interfaces continuously emit state updates for their namespace.
- Action tools do not implicitly subscribe a client to streams.
- Streams do not imply control authority.

## 6. Interface naming rules

All MCP operations shall use namespaced names.

Examples:

- `simulation.start`
- `simulation.stop`
- `simulation.state`
- `simulation.robot_state`
- `simulation.world_state`
- `robot.boot`
- `robot.shutdown`
- `robot.state`
- `robot.move`
- `robot.stop`
- `robot.walk_for_duration`
- `system.state`

All streaming interfaces shall use `.stream` suffixes on the corresponding state surface.

Examples:

- `simulation.state.stream`
- `simulation.robot_state.stream`
- `simulation.world_state.stream`
- `robot.state.stream`
- `system.state.stream`

## 7. Core state models

### 7.1 Simulation runtime state

Simulation runtime state represents the current availability of the simulation environment.

It contains:

- timestamp,
- availability flag,
- running flag,
- world name,
- simulation time,
- optional note,
- optional status details for startup, shutdown, or degraded operation.

Functional semantics:

- `available` means the simulation environment is observable by the MCP,
- `running` means the simulation is currently active rather than merely configured,
- `simulation_time_sec` is present when simulation time is available.

### 7.2 Simulation robot state

Simulation robot state represents the robot as an entity inside the simulation world.

It contains:

- timestamp,
- availability flag,
- world name,
- simulation time,
- robot entity name,
- robot pose in the simulated world,
- optional note.

Functional semantics:

- this is the robot as seen by the simulation environment,
- this state belongs to the `simulation` namespace even though it describes the robot,
- it exists to separate simulated world pose from robot lifecycle and joint state.

### 7.3 Simulation world state

Simulation world state represents the current entity poses in the active world.

It contains:

- timestamp,
- availability flag,
- world name,
- simulation time,
- entity count,
- list of world entities,
- source identifier,
- optional note.

Each world entity contains:

- entity name,
- entity identifier when available,
- entity pose.

### 7.4 Robot state

Robot state represents robot lifecycle and robot-local kinematic state.

It contains:

- timestamp,
- availability flag,
- lifecycle state,
- active flag,
- joint states keyed by joint name,
- optional latest accepted motion command summary,
- optional note.

Observed lifecycle states include:

- `torque_on`
- `torque_off`
- `initializing`
- `finalizing`
- `finalized`
- `servos_rebooting`
- unavailable or unknown state represented as `null`

Functional semantics:

- `active = true` means the robot is in `torque_on`,
- robot state does not need to contain simulated world pose,
- robot state may be partially available when lifecycle or joints are missing.

### 7.5 Motion command state

Motion command state represents the most recent accepted motion command.

It contains:

- stride direction in `x`, `y`, and `z`,
- rotation speed,
- body translation in `x`, `y`, and `z`,
- body rotation as roll, pitch, and yaw,
- gait type,
- timestamp,
- optional note.

All numeric command inputs are normalized values in the inclusive range `[-1.0, 1.0]`.

Supported gait types are:

- `tripod`
- `ripple`
- `wave`

### 7.6 System state

System state represents ROS and MCP integration health.

It contains:

- timestamp,
- overall availability flag,
- ROS runtime availability,
- robot lifecycle channel availability,
- joint-state availability,
- motion-command channel availability,
- simulation channel availability,
- world-state channel availability,
- deployment mode,
- optional list of degraded subsystems,
- optional note.

Functional semantics:

- system state is not a substitute for simulation or robot state,
- `deployment_mode` identifies whether the MCP is currently operating in `real_robot` or `simulation` mode,
- it explains whether the required underlying data sources and command paths are available.

### 7.7 Shared pose and joint models

A pose contains:

- `position` with `x`, `y`, `z`,
- `orientation` with quaternion `x`, `y`, `z`, `w`.

Joint states are returned as a mapping from joint name to:

- `position`,
- `velocity`,
- `effort`.

Each joint-state field may be `null` when that value is not present.

## 8. Request-response interfaces

### 8.1 `simulation.start`

Purpose:

- make the simulation environment available.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Attempts to ensure the simulation environment is running.
- If simulation is already running, returns a structured success or no-op result.
- Waits for simulation-related state to become observable.
- Does not require the robot to reach `torque_on`.
- May make simulated robot state and world state available as a side effect.
- Fails when simulation cannot be started or does not become observable within the timeout.

Output:

- simulation lifecycle result containing:
  action,
  simulation state before,
  simulation state after,
  whether simulation was started by this call,
  running flag at completion,
  message,
  optional log path.

### 8.2 `simulation.stop`

Purpose:

- stop the simulation environment.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Attempts to stop the active simulation environment.
- If simulation is already stopped or unavailable, returns a structured success or no-op result.
- Does not imply robot lifecycle shutdown.
- Waits for simulation to become unavailable or non-running when the environment supports orderly shutdown.

Output:

- simulation lifecycle result with the same general shape as `simulation.start`.

### 8.3 `simulation.state`

Purpose:

- return the latest simulation runtime state.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for simulation runtime data.
- Returns the latest consistent simulation snapshot.
- May return partial information when only some simulation signals are available.

Output:

- simulation runtime state.

### 8.4 `simulation.robot_state`

Purpose:

- return the latest simulated robot state.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for simulated robot pose data.
- Returns the latest consistent simulated robot snapshot.
- This interface is world-oriented and shall not be treated as a substitute for robot lifecycle state.

Output:

- simulation robot state.

### 8.5 `simulation.world_state`

Purpose:

- return the latest simulated world state.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for world-state data.
- Returns the latest world snapshot when available.
- May still return simulation time with zero entities when world-state transport is not yet available.

Output:

- simulation world state.

### 8.6 `robot.boot`

Purpose:

- bring the robot to the `torque_on` lifecycle state.

Input:

- `timeout_sec` with default `120.0`.

Behavior:

- Reads current robot state before acting.
- If the robot is already in `torque_on`, returns success without changing state.
- If the robot is in `torque_off` or `finalized`, requests initialization and waits for `torque_on`.
- If the robot is already `initializing`, waits for `torque_on`.
- If the robot is `finalizing`, waits for `finalized`, then requests initialization and waits for `torque_on`.
- If the robot is `servos_rebooting`, waits for `torque_off`, then requests initialization and waits for `torque_on`.
- Fails if lifecycle state remains unavailable and the MCP cannot complete the sequence.

Output:

- robot lifecycle action result containing:
  action,
  state before,
  state after,
  active flag after completion,
  message,
  optional note.

### 8.7 `robot.shutdown`

Purpose:

- move the robot out of the active `torque_on` state.

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
- Does not require simulation shutdown.

Output:

- robot lifecycle action result with the same general shape as `robot.boot`.

### 8.8 `robot.state`

Purpose:

- return the latest robot lifecycle and joint-state snapshot.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for robot-related runtime data.
- Returns the latest cached robot state even if only partial data is available.
- Marks the snapshot as unavailable only when no robot-local state is present.

Output:

- robot state.

### 8.9 `robot.move`

Purpose:

- publish a single normalized motion command for walking or body motion.

Inputs:

- `stride_x`, `stride_y`, `stride_z`,
- `rotation_speed`,
- `body_x`, `body_y`, `body_z`,
- `body_roll`, `body_pitch`, `body_yaw`,
- `gait_type`.

Defaults:

- all numeric inputs default to `0.0`,
- gait defaults to `tripod`.

Behavior:

- Validates every numeric input against `[-1.0, 1.0]`.
- Validates gait type against the supported gait set.
- Publishes the requested command.
- Returns the normalized command content that was accepted.

Output:

- motion command state.

### 8.10 `robot.stop`

Purpose:

- stop walking and body motion by publishing a zeroed motion command.

Behavior:

- Equivalent to calling `robot.move` with all axes set to zero and gait `tripod`.

Output:

- motion command state for the zeroed command.

### 8.11 `robot.walk_for_duration`

Purpose:

- repeat the same motion command for a fixed duration, optionally followed by an explicit stop command.

Inputs:

- `duration_sec`,
- `publish_hz` with default `5.0`,
- `stop_after` with default `true`,
- the same motion parameters accepted by `robot.move`.

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
  final accepted non-zero command,
  message.

### 8.12 `robot.recording.start`

Purpose:

- begin sampling `robot.state` at a fixed interval.

Input:

- `sample_interval_sec` with default `0.5`.

Behavior:

- Starts a new recording session immediately.
- Captures robot-state snapshots repeatedly until stopped.
- Rejects non-positive sampling intervals.
- Rejects attempts to start a second recording while one is already active.

Output:

- recording status with active flag, sample interval, zero initial sample count, start timestamp, and status message.

### 8.13 `robot.recording.stop`

Purpose:

- stop the active recording session and return the captured samples.

Behavior:

- Stops the active recording loop.
- Waits for the recording worker to finish.
- Returns all captured `robot.state` samples in order.
- Rejects requests when no recording session is active.

Output:

- recorded robot states containing start timestamp, stop timestamp, sampling interval, sample count, and ordered robot-state samples.

### 8.14 `robot.recording.status`

Purpose:

- report whether robot-state recording is active.

Behavior:

- Returns an active status when recording is underway.
- Returns an idle status when no session exists.

Output:

- recording status containing active flag, sample interval or `null`, current sample count, start timestamp or `null`, and status message.

### 8.15 `system.state`

Purpose:

- return the latest system runtime state.

Input:

- `timeout_sec` with default `10.0`.

Behavior:

- Waits briefly for system integration signals.
- Returns the latest snapshot of ROS runtime and channel availability.
- Returns degraded state rather than failing when some subsystems are missing but the MCP remains partially functional.

Output:

- system state.

## 9. Streaming interfaces

Streaming interfaces provide continuous state updates for clients that need observation over time.

### 9.1 General streaming contract

All state streams shall:

- emit an initial snapshot as soon as one is available,
- emit subsequent updates when the underlying state changes,
- permit periodic heartbeats even when state does not change,
- include timestamps on all events,
- identify the stream name and event type,
- terminate cleanly when the client unsubscribes or when the MCP session ends,
- surface degraded or unavailable state as data rather than silently stalling whenever possible.

All state streams may support optional parameters such as:

- `min_update_interval_sec`,
- `heartbeat_interval_sec`,
- `include_unchanged`.

### 9.2 `simulation.state.stream`

Purpose:

- stream simulation runtime state.

Event payload:

- full simulation runtime state snapshot.

Expected update triggers:

- simulation start,
- simulation stop,
- world-name availability changes,
- simulation-time availability changes,
- degraded runtime status changes.

### 9.3 `simulation.robot_state.stream`

Purpose:

- stream the simulated robot state.

Event payload:

- full simulation robot state snapshot.

Expected update triggers:

- simulated robot pose changes,
- simulated robot availability changes,
- simulation-time changes when pose data is coupled to simulation updates.

### 9.4 `simulation.world_state.stream`

Purpose:

- stream simulated world state.

Event payload:

- full simulation world state snapshot.

Expected update triggers:

- world entity pose changes,
- entity count changes,
- world availability changes,
- simulation-time changes when world-state updates are emitted.

### 9.5 `robot.state.stream`

Purpose:

- stream robot lifecycle and joint state.

Event payload:

- full robot state snapshot.

Expected update triggers:

- lifecycle state changes,
- joint-state changes,
- active flag changes,
- latest accepted motion command summary changes.

### 9.6 `system.state.stream`

Purpose:

- stream system runtime and integration health state.

Event payload:

- full system state snapshot.

Expected update triggers:

- ROS runtime availability changes,
- robot lifecycle channel availability changes,
- motion-command channel availability changes,
- simulation channel availability changes,
- world-state channel availability changes,
- deployment-mode changes,
- degraded subsystem changes.

## 10. Validation and error behavior

### 10.1 Input validation

The MCP rejects requests in these cases:

- any motion axis or rotation input outside `[-1.0, 1.0]`,
- unsupported gait type,
- non-positive recording interval,
- non-positive walk duration,
- non-positive walk publish rate.

The MCP also rejects invalid recording lifecycle requests:

- attempt to start a recording when one is already active,
- attempt to stop recording when no recording is active.

### 10.2 Timeout behavior

Timeouts are used to bound waits for:

- simulation availability,
- robot lifecycle transitions,
- simulation robot state availability,
- simulation world-state availability,
- robot state availability,
- system-state availability.

On timeout:

- state-reading tools return the best currently known partial snapshot when possible,
- lifecycle-transition tools fail when the requested terminal state is not reached in time,
- simulation start or stop tools fail when simulation availability does not reach the requested condition in time.

### 10.3 Partial availability

The MCP favors structured partial responses over hard failure for inspection operations.

Examples:

- `simulation.world_state` may return simulation time but zero entities when world-state data is not yet available,
- `robot.state` may return unavailable with a note when lifecycle and joint state are absent,
- `system.state` may report degraded subsystems while the MCP remains partially usable.

### 10.4 Publication semantics

For robot lifecycle and motion commands, successful completion means the MCP accepted the request and published the corresponding control message or lifecycle request. It does not by itself guarantee that all downstream physical or simulated behavior beyond the specified contract has completed, except where the tool explicitly waits for a terminal lifecycle state.

`robot.walk_for_duration` is higher level: it guarantees repeated command publication according to the request and, when enabled, an explicit stop command afterward.

### 10.5 Streaming semantics

Successful stream subscription means the MCP accepted the subscription and will emit events as state becomes available. It does not guarantee a fixed event frequency unless the stream contract explicitly includes heartbeat behavior.

## 11. Observable behavior requirements

### 11.1 Simulation requirements

- The MCP shall expose simulation lifecycle operations separate from robot lifecycle operations.
- The MCP shall expose simulation runtime state separately from robot lifecycle state.
- The MCP shall expose simulated robot state separately from robot lifecycle and joint state.
- The MCP shall expose simulated world state with names, identifiers, and poses when world-state data is available.
- The MCP shall support streaming interfaces for simulation runtime state, simulated robot state, and simulated world state.

### 11.2 Robot requirements

- The MCP shall expose robot boot and shutdown operations separate from simulation start and stop.
- The MCP shall be able to bring the robot to `torque_on` from an available off state.
- The MCP shall tolerate shutdown requests when the robot is already not active.
- The MCP shall expose robot lifecycle and joint state separately from simulated world pose.
- The MCP shall support normalized motion commands with at least the `tripod`, `ripple`, and `wave` gait modes.
- The MCP shall expose a dedicated stop command.
- The MCP shall support bounded walking sequences.
- The MCP shall support robot-state recording and robot-state streaming.
- All `robot` streams shall remain meaningful on a real robot deployment without simulation state.

### 11.3 System requirements

- The MCP shall expose a system namespace for ROS and MCP runtime state that is not owned solely by simulation or robot.
- The MCP shall surface degraded subsystem state explicitly.
- The MCP shall support streaming system state.
- All `system` streams shall remain meaningful on a real robot deployment without simulation state.

### 11.4 Separation requirements

- Simulation state and robot state shall be modeled and exposed as separate state surfaces.
- A client shall be able to inspect simulation state without using robot lifecycle tools.
- A client shall be able to inspect robot state without reading world-state data.
- A client shall be able to subscribe to simulation, robot, and system state streams independently.

## 12. Assumptions visible to callers

The following assumptions are functionally relevant to callers:

- The MCP operates against the current ROS 2 environment.
- Simulated robot pose belongs to the `simulation.robot_state` surface.
- Robot lifecycle and joint state belong to the `robot.state` surface.
- World state belongs to the `simulation.world_state` surface.
- System runtime and channel health belong to the `system.state` surface.
- Request-response tools return snapshots.
- Stream interfaces return continuous updates over time.

## 13. Non-functional exclusions

This specification does not define:

- internal concurrency model,
- specific ROS topic names except where unavoidable for behavior definition,
- transport libraries,
- server transport mode,
- dependency-loading strategy,
- process-supervision approach,
- file layout,
- test strategy,
- implementation ownership boundaries.

Those concerns may change without changing the functional contract described here.
