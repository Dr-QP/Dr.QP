# Spec 05: Python, joy, and launch-test support

- **Status**: proposed
- **Depends on**: [Spec 04](04-cpp-libraries-and-control-plugin.md)
- **Packages**: `drqp_kinematics`, `drqp_brain`, `drqp_launch_testing`,
  `drqp_joy`, `drqp_keyboard_control`, vendored `launch`/`launch_pytest`,
  `sdl3_vendor`
- **Size**: M/L

## Objective

Build the Python/application layer and joy component with Python 3.12, preserve
the repository's patched launch-pytest semantics, and run the focused unit and
small launch tests before introducing Gazebo or MoveIt.

## Public Bazel label contract

| Package                 | Public labels                                                                                                   |
| ----------------------- | --------------------------------------------------------------------------------------------------------------- |
| `drqp_kinematics`       | `:drqp_kinematics`, `:kinematics_tests`                                                                         |
| `drqp_launch_testing`   | `:drqp_launch_testing`, `:launch_testing_tests`                                                                 |
| `drqp_joy`              | `:drqp_joy_component`, `:game_controller_node`, `:joy_tests`                                                    |
| `drqp_brain`            | `:drqp_brain_lib`, `:drqp_brain`, `:drqp_robot_state`, `:drqp_joystick_translator`, `:drqp_imu`, `:brain_tests` |
| `drqp_keyboard_control` | `:drqp_keyboard_control_lib`, `:drqp_keyboard_control`, `:keyboard_control_tests`                               |

Every Python library must list its package sources and runtime dependencies;
every binary must call the same `main` function as its `setup.py` console
entry point. Include `package.xml`, resource-index markers, and
`drqp_brain/launch/bringup.launch.py` as runtime data where setup.py currently
installs them.

## Python dependency lock

Extend a checked-in Bazel requirements input and lock for Python 3.12. Resolve
the direct packages declared by the owning `setup.py` files, including NumPy,
SciPy, python-statemachine, pygame, pytest/pytest-retry, and the IMU hardware
libraries required to import the entry point. Pin hashes for Linux
`amd64`/`x86_64` and `arm64`/`aarch64` wheels or source distributions.

Do not change the ament `install_requires` ranges merely to mirror lockfile
syntax. The lock is Bazel's resolved snapshot; setup.py remains the packaging
contract. If a dependency has no buildable locked artifact on either required
architecture, record the package and reproducer and leave the spec blocked;
never omit an entry point on one architecture.

## Preserve the vendored launch behavior

The workspace's source of truth is
`packages/vendor/launch/source-info.yaml` at revision
`7df946eec4ef5d24c427a16d72c19b368dd643d1`. It fixes launch-pytest retry
re-wrapping so a retried function-scoped launch fixture receives a fresh event
loop. The candidate rules fork otherwise supplies a different `launch` source.

Update the Dr-QP rules fork's `@ros2_launch` repository definition to the
workspace's vendored revision (or apply an equivalent, byte-for-byte reviewed
patch to the same upstream release), then:

- document the override and patch provenance in fork `UPSTREAM.md`;
- ensure the external `launch` and `launch_pytest` targets used by `ros2_test`
  contain the fix;
- compare the forked source/patch with the checked-in vendor source so the two
  build paths cannot drift unnoticed;
- update the workspace's immutable fork pin, integrity, and module lock.

Do not solve this by adding a per-package `conftest.py` retry shim. Preserve the
current fixture rules: function-scoped retries relaunch; module-scoped shared
simulations are crash-safe but are not made relaunch-safe by `pytest-retry`.

## SDL and joy mapping

Build SDL 3.4.4 from immutable commit
`5848e584a1b606de26e3dbd1c7e4ecbc34f807a6`, matching
`sdl3_vendor/CMakeLists.txt`, with the same disabled audio/camera/examples/GPU/
render/test/video options. Export only the headers/shared library required by
`drqp_joy`.

Translate `drqp_joy_component` as a C++20 shared rclcpp component, preserve its
registered class and `game_controller_node` executable, generated interface
dependency, and existing warning policy. The focused utility test must remain
headless and use its existing Catch2 source.

## Tests to wire first

### Launch infrastructure gate

Run all functional tests under:

- `packages/runtime/drqp_launch_testing/test/test_process_exit_codes.py`;
- `packages/runtime/drqp_launch_testing/test/shutdown_behavior/`.

Specifically prove combo 5 relaunches on retry and combo 6 does not reuse a
closed/duplicated event loop. Keep combo 4's documented module-scope limitation.
These tests form `:launch_testing_tests` and must pass before other launch tests
are enabled.

After that gate, add
`//packages/runtime/drqp_control:control_launch_test` for the existing
`drqp_control/test/test_a1_16_hardware_interface.py`. Preserve its 300-second
timeout, module-scoped launch fixture, class-scoped ROS client setup,
`shutdown=True` process check, mock-servo address, controller/action/state
assertions, and clean-exit helper. Do not replace it with the rules-fork mock
test from Spec 03.

### Python and joy suites

- `:kinematics_tests`: every `drqp_kinematics/test/test_*.py` except the three
  ament lint files (`copyright`, `flake8`, `pep257`).
- `:brain_tests`: every non-lint `drqp_brain` test, including the existing
  `test_brain_node.py` and `test_robot_state_node.py` launch tests. Include the
  test package/support files as declared deps/data. Do not run Gazebo or the
  real MoveIt stack in this spec.
- `:keyboard_control_tests`: every non-lint keyboard test. Configure SDL's
  documented dummy/headless environment for tests; do not delete GUI tests.
- `:joy_tests`: existing `TestGameControllerUtils.cpp` without physical input
  devices.

Launch tests must preserve their fixture scope, `shutdown=True` placement,
pytest-retry markers, bounded readiness checks, and
`drqp_launch_testing.assert_processes_exited_cleanly` assertions. Tag DDS tests
`requires-network` and give each test an isolated ROS domain.

### Entry-point checks

Run each of the four `drqp_brain` binaries and the keyboard binary through a
small smoke harness that starts the process with its supported non-hardware
arguments/environment, waits for initialization or help output, and shuts it
down cleanly. Hardware-only IMU access must be injected/mocked using the same
boundary as existing tests; do not access GPIO/I2C in CI.

The `drqp_brain` full MoveIt/Gazebo behavior is deferred to Specs 06–07, but
the binary must import and reach its bounded initialization path here.

## Verification

```bash
scripts/bazel.sh test //packages/runtime/drqp_launch_testing:launch_testing_tests
scripts/bazel.sh test //packages/runtime/drqp_control:control_launch_test
scripts/bazel.sh test \
  //packages/runtime/drqp_kinematics:kinematics_tests \
  //packages/runtime/drqp_joy:joy_tests \
  //packages/runtime/drqp_brain:brain_tests \
  //packages/simulation/drqp_keyboard_control:keyboard_control_tests
scripts/with-ros-env.sh colcon build --symlink-install --packages-up-to \
  drqp_brain drqp_keyboard_control
scripts/with-ros-env.sh colcon test --packages-select \
  drqp_launch_testing drqp_kinematics drqp_joy drqp_brain \
  drqp_keyboard_control
scripts/with-ros-env.sh colcon test-result --verbose
```

Repeat Bazel tests after `clean --expunge`. Record collected test names/counts,
entry-point outcomes, Python lock/artifact selection by architecture, SDL
provenance, launch-fork SHA, and per-architecture Bazel plus colcon-oracle
results in `evidence/05-python-launch.md`.

## Allowed files

- BUILD files and narrowly required shared test/runtime lookup helpers in the
  owning packages;
- `packages/runtime/drqp_control/BUILD.bazel` only to add the reserved
  `:control_launch_test` target;
- Bazel Python requirements/lock and SDL adapters;
- rules-fork launch provenance/patch/tests and workspace fork pin files;
- this program's evidence/status documentation.

Do not edit Gazebo/MoveIt BUILD files, migrate `drqp_robot_mcp`, or change
launch fixture scope/retry markers.

## Acceptance criteria

- [ ] Python 3.12 dependencies are hash-pinned for both CI architectures and
      do not leak user-site packages.
- [ ] Every listed Bazel build/test and bounded entry-point smoke check passes
      natively on both `amd64` and `arm64`.
- [ ] All public libraries, component/node, and five console-entry labels build
      and their bounded smoke checks pass.
- [ ] Non-lint kinematics, brain, keyboard, and joy tests pass from shared
      sources under Bazel and colcon.
- [ ] Vendored launch combo 5/6 retry tests and process-exit helper tests pass
      under Bazel without per-package shims.
- [ ] Package resources and entry points resolve from runfiles/ament setup, not
      the source checkout or workspace install tree.
- [ ] `evidence/05-python-launch.md` contains the required handoff data.
