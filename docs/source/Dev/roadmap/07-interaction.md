# Phase 7 — Communication, LLM brain, and pet personality

This is where Dr.QP stops being a platform and starts being a _pet_. The phase has three layers
that can be built mostly independently: **voice I/O**, an **LLM agent brain**, and a **behavior
engine** that keeps the robot alive-feeling even when nobody is talking to it.

The critical head start: `drqp_robot_mcp` already exposes robot control as MCP tools. The LLM
brain is, architecturally, "an agent loop wired to those tools" — the pattern is proven, the tool
surface just needs to grow (navigation, camera, speech).

## Architecture

```text
        on robot (Pi)                          │            off-board (home server / cloud)
                                               │
 mic array ─▶ wake word (openWakeWord) ─▶ VAD ─┼─▶ ASR (whisper.cpp / cloud) ─▶ text
                                               │                                 │
 speaker ◀─ TTS (Piper) ◀──────────────────────┼──────────── LLM agent loop ◀────┘
                                               │      (Claude via API, MCP tools:
 /robot/* topics, Nav2 actions, camera ◀──────▶│       go_to, look, say, emote,
 behavior engine (state machine, idle life) ───┼──────▶ snapshot, status, …)
```

- **Wake word + VAD on the Pi** (openWakeWord/Porcupine class — tiny CPU). Raw audio never
  streams continuously off the robot; only post-wake utterances do. That is both a privacy and a
  bandwidth decision.
- **ASR**: whisper.cpp `small`-class on the home server, or a cloud STT — behind one ROS
  service/action so backends swap freely.
- **TTS**: Piper on the Pi or server (low latency, pleasant voices, fully local). The pet also
  needs _non-verbal_ audio — chirps and trills are more pet-like than sentences, and cheaper.
- **LLM brain**: an agent loop (Claude API) with the MCP tool surface. It is _not_ in the safety
  path: every tool it calls goes through the same validated command topics with the same state
  machine, watchdogs, and kill switch. The LLM can suggest; the robot's reflexes decide.
- **Grounding**: system prompt includes the semantic map (rooms, dock), robot state, battery, and
  recent events; tools return structured observations (`where_am_i`, `snapshot` + optional
  vision-model description).

## The behavior engine (what makes it a pet)

An LLM in the loop for every wag would be slow, expensive, and lifeless between requests. A local
behavior layer — the repo already has `python-statemachine` and a `create-state-machine` skill —
runs the day-to-day:

- **Idle behaviors**: settle, look around (body pose, not walking), stretch (a scripted
  trajectory like the existing init sequence), nap when battery is low, wander occasionally.
- **Reactive behaviors**: turn toward loud sounds (mic array direction-of-arrival), greet a
  detected person (camera person detection — a tiny model on the Pi/accelerator), flinch on IMU
  disturbance (being picked up → go limp / `torque_off` for safety — _pick-up detection is a
  safety feature_).
- **Emotes as a first-class command**: `emote(happy | curious | sleepy | alert)` maps to body
  poses, gait flourishes, sounds, and (future) LED eyes. Exposed as an MCP tool so both the
  behavior engine and the LLM share the same vocabulary.
- The LLM is invoked for _conversation and decisions_, on wake word or on behavior-engine
  escalation ("unknown person at door"), and its outputs are behavior-engine goals.

## Milestones

1. Hardware: mic array (e.g. 2–4-mic I2S/USB) + small speaker; audio in/out ROS nodes.
2. Wake word → ASR → text on a topic; TTS service; round-trip "parrot" demo.
3. MCP tool growth: `robot.say`, `robot.emote`, `robot.snapshot_describe`, `robot.go_to`
   (Phase 6), `robot.status`.
4. LLM agent service with the tool loop + grounding context; conversation demo: "go to the
   kitchen and tell me what you see."
5. Behavior engine with 3–4 idle behaviors, person greeting, pick-up reflex.
6. Personality tuning pass: latency budget (<2 s wake-to-first-sound via streaming ASR/TTS),
   voice/sound design, motion character.

## Risks and notes

- **Latency is the product.** A pet that freezes for 5 s feels broken; fill LLM thinking time
  with behavior-engine acknowledgments (head tilt, chirp) immediately on wake.
- Walking servos are loud; mic capture during locomotion will need the wake word to tolerate it
  (test early), or the pet simply stops to listen — which is charming anyway.
- Cloud vs local LLM is a config choice behind one interface; design for both from day one.
- Privacy: local wake word, explicit indicator (sound/LED) when streaming audio, no always-on
  camera uploads.

## Definition of done

- Wake the robot by name, ask it to go somewhere and report, get a spoken answer; robot behaves
  autonomously and safely when ignored. Behaviors and tools covered by sim-side tests where
  feasible (behavior engine and tool loop are hardware-independent).

Next: {doc}`08-feet-terrain`.
