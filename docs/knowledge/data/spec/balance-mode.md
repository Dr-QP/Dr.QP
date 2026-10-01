---
type: spec
description: Balance mode as a stationary, reachability-bounded IMU posture hold — entry conditions, exits, and saturation reporting.
generated:
  by: claude-code/opus-5
  at: 2026-09-25T00:00:00Z
sources:
- id: brain
  resource: packages/runtime/drqp_brain/drqp_brain/brain_node.py
- id: program
  resource: docs/agents/specs/balance-mode-safety/README.md
---

# Balance mode

Balance mode is a stationary, reachability-bounded body-posture hold driven by
the IMU. It does not support walking at the same time; that needs contact-aware
locomotion, which does not exist yet. The implementation is in
`drqp_brain/brain_node.py` (`process_balance_mode`, `_run_loop`,
`_constrain_balance_correction`) and `balance_controller.py`. The contract comes
from the balance-mode-safety program (`docs/agents/specs/balance-mode-safety/`),
which is in progress. Update this stub as that program lands.

## Requirements

### Requirement: Guarded entry

A `true` on `/robot/balance_mode` (transient-local `std_msgs/Bool`) SHALL enable
balance mode only when all of these hold:

- `enable_imu_balance` is true.
- The robot state is `torque_on`.
- A fresh IMU tilt is available, no older than `imu_balance_timeout_sec`
  (default 1.0 s).

When enabled, the current tilt becomes the target posture and any operator
motion is discarded. Otherwise the request SHALL be ignored.

#### Scenario: Enable without IMU

- **GIVEN** `torque_on` and no `/imu/data` received in the last second
- **WHEN** `true` is published on `/robot/balance_mode`
- **THEN** balance mode stays disabled

### Requirement: Stationary hold

While balance mode is enabled, the robot SHALL NOT walk. Stride and rotation are
zero, incoming `MovementCommand`s are ignored, and the body is counter-rotated
toward the target tilt. The gain is `imu_balance_gain` (default 2.0), clamped to
`imu_balance_max_tilt_rad` (default 0.15 rad).

#### Scenario: Tilted surface

- **GIVEN** balance mode is enabled on level ground
- **WHEN** the support surface tilts by 0.1 rad
- **THEN** the body counter-rotates toward the original posture and the feet
  stay in place

### Requirement: Automatic exit

Balance mode SHALL disable itself, and discard any stored motion command so that
nothing replays, when any of these happens:

- `false` is received on `/robot/balance_mode`.
- The IMU reports unavailable orientation (negative covariance or a zero
  quaternion).
- IMU data goes stale.
- The robot leaves `torque_on`.

#### Scenario: IMU dropout

- **GIVEN** balance mode is enabled
- **WHEN** `/imu/data` stops for longer than `imu_balance_timeout_sec`
- **THEN** balance mode disables and the robot holds a neutral stationary stance

### Requirement: Reachability bound and saturation diagnostic

The balance correction SHALL be scaled down until every foot target is
reachable. The brain SHALL log a
`stationary_balance_correction_saturated:<legs> scale=<s>` warning whenever the
set of constrained legs changes, throttled to once per 5 s.
