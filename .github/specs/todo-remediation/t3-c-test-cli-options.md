# T3-C · Add CLI argument parsing to `TestXYZrobotServo`

**Severity:** Low (developer ergonomics)
**Agent turns:** 3
**File:** `packages/runtime/drqp_a1_16_driver/test/TestXYZrobotServo.cpp:49`

## Background

The hardware integration test for the A1-16 servo has a `TestOptions` struct with settings
that control whether real hardware is used, which serial port to target, and whether to reset
torque on exit:

```cpp
// TODO(anton-matosov): Add command line options for these options
struct TestOptions {
  std::string serialAddress = "/dev/ttySC0";
  bool useRealHardware = false;
  bool resetTorque = false;
  bool skipReturnToNeutral = false;
};
static TestOptions testOptions;
```

Currently the only way to change these is to recompile. Engineers testing against real hardware
must edit the source and rebuild.

## Goal

Parse the `TestOptions` fields from `argv` in `main()`, allowing `--serial-address`,
`--use-real-hardware`, `--reset-torque`, and `--skip-return-to-neutral` flags without
recompiling.

## Acceptance criteria

- [ ] All four options are configurable from the command line.
- [ ] Unknown arguments are reported and cause a non-zero exit code.
- [ ] `--help` prints a usage summary.
- [ ] Default values are unchanged (CI, which passes no flags, continues to work).
- [ ] The TODO comment is removed.

## Implementation approach

### Turn 1 — Choose a parsing library

Check what is already available in the build graph. Options:

- `cxxopts` (header-only, likely available).
- `boost::program_options` (already a transitive dependency via Boost ASIO).
- Manual `argv` loop (no new dependency).

Prefer re-using an existing dependency to avoid adding a new one.

### Turn 2 — Implement parsing in `main()`

Call the chosen parser before `testing::InitGoogleTest`. Pass `argc`/`argv` through. Populate
`testOptions`. Print usage if `--help` is passed.

### Turn 3 — Verify

Build the test. Run without flags (verify defaults), run with `--use-real-hardware` (verify
override), run with an unknown flag (verify error). Update any CI scripts that invoke this test
binary directly to confirm they still work with no arguments.

## Files to modify

| File                                                            | Change                            |
| --------------------------------------------------------------- | --------------------------------- |
| `packages/runtime/drqp_a1_16_driver/test/TestXYZrobotServo.cpp` | Add CLI parsing in `main()`       |
| `packages/runtime/drqp_a1_16_driver/CMakeLists.txt`             | Add new link dependency if needed |

## Dependencies

None. Purely additive; default behaviour is unchanged.
