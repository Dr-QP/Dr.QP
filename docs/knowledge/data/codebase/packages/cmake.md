---
type: codebase
description: Shared CMake helpers included by relative path from the C++ and launch-test packages — Catch2 test registration, clang source coverage, clang-format config, Boost depends patch, isolated launch tests — plus the LLVM coverage exporter.
source: packages/cmake
source_digest: sha256:c4fcf1bdc1c1f05e26fdd1ff836d0ff29186ca30c52d146033592a72b78d1973
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/cmake
---

# packages/cmake

This is not a colcon package: a `COLCON_IGNORE` file hides it. Other packages
`include(../../cmake/<File>.cmake)` by relative path.

## Public surface

- **`Catch2Extras.cmake` →
  `add_catch2_unit_test(TARGET [EXECUTABLE] [RESULT_FILE] [OUTPUT_FILE] [EXTRA_COMMAND])`:**
  registers a Catch2 binary with `ament_add_test`. It writes a JUnit report to
  `AMENT_TEST_RESULTS_DIR` and sets `LLVM_PROFILE_FILE=<exe>.profraw`.
- **`ClangCoverage.cmake`:** the option `DRQP_ENABLE_COVERAGE` (default OFF),
  and functions that add the `-fprofile-instr-generate -fcoverage-mapping`
  flags:
  - `drqp_library_enable_coverage`: compile flags.
  - `drqp_test_enable_coverage`: link flags only, so the test's own code is
    ignored.
  - An executable variant: compile and link flags.
- **`ClangFormatConfig.cmake`:** when `BUILD_TESTING` is on, it sets
  `ament_cmake_clang_format_CONFIG_FILE` to the repo-root `.clang-format`.
  Configuration fails if that file is missing.
- **`PatchDepends.cmake` → `patch_depends_variables()`:** rewrites `boost` to
  `Boost` in every `*_DEPENDS` variable.
- **`RosIsolatedLaunchTest.cmake` →
  `drqp_add_ros_isolated_launch_test(path …)`:** runs `ament_add_pytest_test`
  through `ament_cmake_ros`'s `run_test_isolated.py`.
- **`llvm-cov-export-all.py <base_path>`:** for every `*.profraw` under
  `base_path`, the binary is the same path without the suffix. The script merges
  the profile with `llvm-profdata merge -sparse` and exports it with
  `llvm-cov export --format lcov` to `<bin dir>/coverage/<binary>/lcov.info`.

## How it works

The users are:

- `drqp_serial`, `drqp_a1_16_driver`, and `drqp_joy` include the coverage,
  format, and Catch2 helpers. `drqp_serial` also includes `PatchDepends`.
- `drqp_control/test` and `drqp_gazebo/test` register their launch tests through
  the isolated runner.
- CI builds with `-D DRQP_ENABLE_COVERAGE=ON` and then runs
  `llvm-cov-export-all.py ./build` before the Codecov upload.

## Depends on

- `ament_cmake_test`, `ament_cmake_pytest`, `ament_cmake_ros`, and the
  clang/LLVM toolchain.

## Invariants & gotchas

- The include paths are relative, so moving a package to a different depth under
  `packages/` breaks its `include(...)` lines.
- With coverage enabled, a non-Clang compiler fails configuration
  (`__check_compatibility`).
- The Catch2 helper names the profile `<exe>.profraw`. The exporter depends on
  that naming to find the binary.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/cmake/Catch2Extras.cmake:2` — `add_catch2_unit_test`
- `packages/cmake/ClangCoverage.cmake:3` — `DRQP_ENABLE_COVERAGE`
- `packages/cmake/ClangCoverage.cmake:15` — `drqp_library_enable_coverage`
- `packages/cmake/PatchDepends.cmake:2` — `patch_depends_variables`
- `packages/cmake/RosIsolatedLaunchTest.cmake:1` —
  `drqp_add_ros_isolated_launch_test`
- `packages/cmake/llvm-cov-export-all.py:18` — `llvm-profdata merge`
- `.github/workflows/ci.yml:176` — coverage build flag
