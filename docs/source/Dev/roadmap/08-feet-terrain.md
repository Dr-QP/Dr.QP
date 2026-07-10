# Phase 8 — Feet sensors and terrain adaptation

Everything until now assumed flat floors and trusted the gait plan ("leg 3 _should_ be in stance
now"). Real homes have rugs, thresholds, toys, and socks. Ground-truth **foot contact** upgrades
balance, odometry, and locomotion — and it is the single most valuable observation for the RL
phase.

## Sensor plan (in line with the hardware roadmap)

1. **Now — binary contact switches / FSRs.** A microswitch or force-sensitive resistor in each
   foot tip, read by a small ADC/GPIO board (e.g. an RP2040 on the existing I2C/UART budget → one
   `feet_contact` ROS node publishing per-leg booleans + raw force). Simple, robust, cheap.
2. **Later — servo-side proprioception.** The planned upgrade to Dynamixel **XC430-T240BB-T**
   class servos brings current/load feedback per joint. Joint load + kinematics = contact force
   _estimation_ without foot electronics, plus torque telemetry the A1-16 bus simply does not
   offer. The software contract below is designed so this swap changes the _source_, not the
   consumers.

**Contract**: a `FootContacts` message (per leg: `in_contact`, `force_estimate`, `confidence`)
published at gait rate. Simulation provides it via gz-sim contact sensors on the tibia tips —
so every consumer is developed and CI-tested in Gazebo before any soldering.

## What contact data unlocks (consumers, in order)

1. **Leg odometry v2 (Phase 3 upgrade)**: use _measured_ stance instead of gait-phase-assumed
   stance; ignore slipping feet (contact + unexpected motion). Expect a large drift improvement
   on rugs.
2. **Touchdown adaptation**: end swing when contact fires, not when the spline ends. This single
   rule handles thresholds and rug edges: step height stops being a guess. It fits cleanly into
   `WalkController.next_step_targets` as a per-leg phase adjustment.
3. **Stability supervision**: stance polygon + IMU tilt = "am I about to tip?" → freeze/lower
   body reflex in the brain (fast path, no LLM, no Nav2).
4. **Terrain classification (nice-to-have)**: contact timing + IMU vibration signature
   distinguishes hard floor / rug / grating; gait parameters adapt (slower, higher steps on rugs).
5. **RL observations (Phase 9)**: contact booleans are a standard, high-value input in every
   published legged-locomotion policy.

## Milestones

1. `FootContacts` message + gz-sim contact sensors + bridge; sim publishes truthful contacts
   during walking (CI test: contact pattern matches gait phase on flat ground).
2. Leg odometry consumes measured stance (behind a parameter); drift benchmark re-run and
   compared.
3. Touchdown adaptation in the walk controller; sim test world with a 1 cm threshold and a
   soft-rug patch (compliant surface) — robot crosses both without tipping.
4. Hardware: foot sensor prototype on one leg → all six; calibration + debounce; re-run
   benchmarks on real rugs.
5. Stability supervisor reflex with tests (balance-board world already exists in `drqp_gazebo`).

## Risks and notes

- Foot electronics on a walking robot live a hard life: strain relief and connector choice matter
  more than sensor choice. Budget for two mechanical iterations of the foot tip (3D-printed —
  the CAD pipeline exists).
- Wiring 6 feet through moving joints is the real cost of option 1 — one more argument to keep
  the software contract source-agnostic so the servo upgrade can retire the wiring.
- Keep consumers tolerant of missing contact data (sensor failure → fall back to gait-phase
  stance assumption, log loudly).

## Definition of done

- Robot crosses a doorway threshold and a rug edge in sim (CI) and on hardware without manual
  gait tuning; odometry drift measurably improved; stability reflex demonstrated on the balance
  board.

Next: {doc}`09-rl-locomotion`.
