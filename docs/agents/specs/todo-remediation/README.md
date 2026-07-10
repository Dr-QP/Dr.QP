# TODO Remediation Specs

Audit date: 2026-06-17. 38 raw TODO/FIXME/XXX annotations found across the codebase.

## Disposition summary

| Tier                    | Count | Action                                 |
| ----------------------- | ----- | -------------------------------------- |
| Drop — vendored/meta    | 29    | No action; owned by upstream           |
| T1 — address soon       | 2     | High impact, address next sprint       |
| T2 — address when ready | 3     | Medium impact, schedule within quarter |
| T3 — housekeeping       | 4     | Low effort, opportunistic              |

## Interaction with the locomotion migration

Two items touch code the [locomotion migration](../locomotion/README.md) rewrites: **T1-A**
(rotation assertion in `test_walk_controller.py`) and **T2-C** (rotation branch in the gait
plotter). Locomotion specs 02 and 06 rework the walk-controller API and delete the per-leg
rotation transform entirely. Land T1-A/T2-C **before** that migration (quick wins on the current
code), or fold their intent into specs 02/06 — do not do both. Everything else here is
independent of the migration.

## Effort unit

Estimates use **agent turns** — one agent turn is one focused agent session (research → implement → verify) operating without human intervention. A simple file edit is 1 turn; a full TDD Red/Green/Refactor cycle across multiple files is ~5–8 turns.

## Specs

| ID                                               | File                                                 | Agent turns | Spec                                             |
| ------------------------------------------------ | ---------------------------------------------------- | ----------- | ------------------------------------------------ |
| [T1-A](./t1-a-fix-rotation-test.md)              | `drqp_brain/test/test_walk_controller.py`            | 3           | Fix commented-out rotation assertion             |
| [T1-B](./t1-b-gmock-hardware-interface-tests.md) | `drqp_control/CMakeLists.txt`                        | 10          | Add gmock unit tests for hardware interface      |
| [T2-A](./t2-a-unix-serial-options-parsing.md)    | `drqp_serial/src/UnixSerial.cpp`                     | 4           | Parse `transferConfig` instead of hardcoding 8N1 |
| [T2-B](./t2-b-dynamic-joint-state-test.md)       | `drqp_control/test/test_a1_16_hardware_interface.py` | 3           | Assert position via `dynamic_joint_states` topic |
| [T2-C](./t2-c-rotation-viz-decoupling.md)        | `docs/source/notebooks/plotting/gaits.py`            | 5           | Remove rotation-gait branch from plotter         |
| [T3-A](./t3-a-package-description.md)            | `drqp_control/package.xml`                           | 1           | Fill in placeholder description                  |
| [T3-B](./t3-b-serial-player-optimization.md)     | `drqp_serial/src/SerialPlayer.cpp`                   | 2           | Position-index vs erase-from-front               |
| [T3-C](./t3-c-test-cli-options.md)               | `drqp_a1_16_driver/test/TestXYZrobotServo.cpp`       | 3           | Add CLI argument parsing to hardware test        |
| [T3-D](./t3-d-ik-notebook-diagram.md)            | `docs/source/notebooks/…`                            | 2           | Update IK diagram to match current leg geometry  |
| [DROP](./drop-vendored-meta.md)                  | various                                              | 0           | Vendored and meta TODOs — no action              |
