# T1-B · Add gmock unit tests for the A1-16 hardware interface

**Severity:** High
**Agent turns:** 10
**File:** `packages/runtime/drqp_control/CMakeLists.txt:95`

## Background

`drqp_control` implements a `ros2_control` hardware interface plugin. The package already
declares `ament_cmake_gmock` as a test dependency (`package.xml:36`) but the CMakeLists has
only a placeholder:

```cmake
#TODO: Add gmock tests for hardware interface from
# https://github.com/ros-controls/ros2_control/blob/jazzy/hardware_interface/CMakeLists.txt
```

The existing test (`test_a1_16_hardware_interface.py`) is a full-stack integration test that
requires a running ROS 2 system, a URDF, and the controller manager. It catches system-level
bugs but cannot isolate hardware-interface logic from ROS infrastructure, and it cannot run in
a plain unit-test environment (e.g. in CI without a launched system).

## Goal

Add a gmock-based C++ unit-test suite that exercises the hardware interface class in isolation,
with the serial driver replaced by a mock.

## Acceptance criteria

- [ ] At least the following test cases exist and pass:
  - Hardware interface initialises successfully when given a valid URDF/config.
  - `on_init` returns `CallbackReturn::ERROR` when mandatory YAML parameters are missing.
  - `on_activate` transitions state to `ACTIVE`.
  - `on_deactivate` transitions state to `INACTIVE`.
  - `write()` with a valid command produces the expected byte sequence on the mock serial port.
  - `read()` with a valid response correctly populates joint state interfaces.
- [ ] Tests compile with `colcon build --packages-select drqp_control --cmake-args -DBUILD_TESTING=ON`.
- [ ] Tests pass with `colcon test --packages-select drqp_control`.
- [ ] No new warnings at `-Wall -Wextra`.

## Implementation approach

### Turn 1 — Study upstream reference
Read `ros2_control/hardware_interface/test/` (referenced in the TODO comment) to understand
the test fixture pattern: `MockHardwareInterface`, `HardwareInterfaceTestSuite`, how lifecycle
transitions are driven without a full ROS 2 stack.

### Turn 2 — Identify seams in the hardware interface
Read the current implementation headers and sources in `drqp_control/`. Identify:
- Where the serial driver is injected (constructor? `on_init`?).
- What types are used (`ISerial`, `XYZrobotServo`, etc.).
- Whether a mock-friendly interface already exists in `drqp_serial` or `drqp_a1_16_driver`.

### Turn 3 — Create or extend a mock serial interface
If no mock/fake `ISerial` exists, create one in `drqp_serial` or as a test-only header in
`drqp_control/test/`. The mock should capture written bytes and allow injecting read bytes.
Use gmock `MOCK_METHOD` macros.

### Turns 4–7 — Write test cases (TDD Red/Green cycle)
Implement one test class per concern:
- `HardwareInterfaceInitTest` — init/config parsing.
- `HardwareInterfaceLifecycleTest` — activate/deactivate state transitions.
- `HardwareInterfaceReadWriteTest` — command/state round-trips.

### Turn 8 — Wire CMakeLists
Add `ament_add_gmock(test_a1_16_hardware_interface_unit …)` with correct link libraries.
Mirror the structure from the upstream reference.

### Turn 9 — CI check
Run `colcon build` and `colcon test` locally; confirm all tests pass and no unrelated tests
regress.

### Turn 10 — Refactor / cleanup
Remove any dead code introduced during exploration. Ensure test helpers are in a reusable
location if they could be shared with future hardware interface tests.

## Files to modify / create

| File | Change |
|------|--------|
| `packages/runtime/drqp_control/CMakeLists.txt` | Add `ament_add_gmock` target; remove placeholder TODO |
| `packages/runtime/drqp_control/test/test_a1_16_hardware_interface_unit.cpp` | New file — gmock test suite |
| `packages/runtime/drqp_serial/include/…/MockSerial.h` (if needed) | New file — gmock mock for `ISerial` |

## Dependencies

- T2-B (optional): the integration test improvement can be done independently but is related.
- Requires understanding of the A1-16 hardware interface implementation details before starting.
