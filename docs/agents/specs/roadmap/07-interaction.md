---
id: RM-07
title: Voice, LLM agent brain, and pet behavior engine
status: proposed
depends_on: [RM-04] # conversation/nav tools additionally need RM-06
packages:
  [
    drqp_robot_mcp,
    drqp_interfaces,
    'NEW: drqp_audio',
    'NEW: drqp_agent',
    'NEW: drqp_behavior',
  ]
---

# RM-07 — Interaction

## Objective

Wake-word voice interaction with an LLM agent grounded in robot state and MCP tools, plus a local
behavior engine providing idle/reactive pet behaviors and safety reflexes. Three independently
landable tracks.

## Track A — Audio I/O (`drqp_audio`)

- Hardware: I2S/USB mic array + speaker on the Pi (document chosen parts in hardware docs).
- Nodes: `audio_capture` (wake word via openWakeWord + VAD; publishes utterance WAV/PCM on
  `/audio/utterance` only after wake), `speech_synthesis` (service/action `robot/say` → Piper TTS
  → speaker; also `robot/play_sound` for non-verbal chirps from an asset dir).
- ASR: `speech_recognition` action node wrapping whisper.cpp (local or remote host) →
  `/speech/text` (`std_msgs/String` + confidence). Backend selection by parameter.
- Acceptance: parrot launch test with injected WAV (no hardware): wake → ASR → text → TTS file
  output asserted; wake word false-accept/reject measured with servo-noise recordings.

## Track B — LLM agent (`drqp_agent`)

- Agent loop service (Claude API via `anthropic` SDK; model/backend configurable) consuming
  `/speech/text`, calling MCP tools, replying through `robot/say`.
- Grounding context assembled per turn: `/robot_state`, `/battery_state`, `where_am_i` (RM-05),
  recent events, `locations.yaml` names.
- Tool surface (extend `drqp_robot_mcp/server.py`): existing `robot.*` + `robot.say`,
  `robot.emote`, `robot.go_to` (RM-06), `robot.camera.snapshot` (RM-04),
  `robot.snapshot_describe` (vision model, off-board OK), `robot.status`.
- Safety: agent publishes only semantic commands/goals through existing validated topics/actions;
  it cannot bypass state machine, mux priorities, or watchdogs. Tool-call rate limited.
- Acceptance: scripted-conversation test with mocked LLM backend: text in → correct tool calls →
  spoken reply; live end-to-end demo documented.

## Track C — Behavior engine (`drqp_behavior`)

- `python-statemachine`-based (see `create-state-machine` skill) node above the robot state
  machine. States: `idle`, `observing`, `interacting`, `resting`, `alarmed`.
- Behaviors: idle sway/look-around via body-pose `MovementCommand` (no walking), stretch
  (scripted trajectory like init sequence), low-battery → rest/dock-seek (dock nav needs RM-06),
  sound-direction orient (mic array DOA, optional), person greeting (lightweight person detector
  on `/camera/image_raw`, ≤ 2 Hz).
- **Pick-up reflex (safety)**: IMU free-fall/lift signature ⇒ publish `kill_switch_pressed` or a
  new `picked_up` event → torque-safe behavior; must work without Tracks A/B.
- `robot.emote(name)`: mapping table emotion → {body pose sequence, sound, gait flourish};
  exposed as MCP tool and internal API.
- Acceptance: sim launch tests — idle behaviors emit only body-pose commands (no stride);
  pick-up reflex fires on injected IMU profile; emote executes and returns to neutral.

## Constraints

- No continuous raw audio/video off the robot; only post-wake utterances and explicit snapshots.
- All LLM/ASR heavy compute placeable off-board via config; robot-side must degrade to
  behavior-engine-only when offline.
- Latency budget: wake → first audible acknowledgment ≤ 1 s (behavior ack), ≤ 4 s full reply.
