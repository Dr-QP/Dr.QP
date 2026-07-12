# Spec 03: Consistent ROS package lint gates

- **Status**: proposed
- **Depends on**: 01
- **Size**: M

## Objective

Make ament lint registration explicit and consistent across every first-party ROS package, then
make the full repository lint command reproduce those package gates.

## Current gaps to fix

- `drqp_gazebo/package.xml` declares `ament_lint_auto` and `drqp_lint_common`, but its CMake never
  calls `ament_lint_auto_find_test_dependencies()`.
- `drqp_robot_mcp/package.xml` declares copyright, flake8, and pep257 dependencies, but its test
  directory has no corresponding lint tests.
- The manual whole-tree ament_flake8 check currently reports 26 findings that normal package
  registration does not reliably expose.
- Ruff excludes `packages/vendor`, while `python-lint-check.sh` includes it. The difference may be
  intentional, but it is not expressed as a vendor policy.
- Only four C++ packages include `ClangFormatConfig.cmake`; package coverage currently depends on
  package contents and handwritten CMake rather than a tested convention.

## Design

1. Inventory every package by build type and source languages.
2. For `ament_cmake` packages, require the standard guarded pattern:

   ```cmake
   if(BUILD_TESTING)
     # Include root clang-format configuration when the package owns C/C++.
     find_package(ament_lint_auto REQUIRED)
     ament_lint_auto_find_test_dependencies()
   endif()
   ```

3. For `ament_python` packages, provide one shared lint-test template or generated helper for
   copyright, ament_flake8, and pep257. Avoid copy/paste drift while retaining normal colcon test
   discovery.
4. Add an audit test that fails when a package declares a lint dependency without registering the
   check, or registers a lint check without declaring its dependency.
5. Keep Ruff as formatter and fast linter; keep ament_flake8 as the ROS compatibility gate. Record
   known rule differences instead of pretending the tools are interchangeable.
6. Adopt a vendor policy:
   - repository formatters do not rewrite imported vendor source;
   - vendor packages may retain upstream lint tests;
   - first-party full-tree lint output reports vendor failures separately and does not silently
     change policy based on which entry point was used.
7. Fix the current baseline findings in a dedicated style-only commit after gate registration is
   proven, so behavioral changes are not mixed into mechanical cleanup.

The package audit should be structural and fast; it must not infer registration solely from
`package.xml`. For CMake, inspect generated CTest/ament metadata or validate the required call. For
Python, verify collected lint tests.

## Test plan

- Write failing audit cases for the current Gazebo and robot MCP gaps before fixing them.
- Run incremental builds with `scripts/with-ros-env.sh ... --packages-up-to` for affected packages.
- Run package-specific tests with `--packages-select drqp_gazebo drqp_robot_mcp` and inspect
  `log/latest_test/<package>/stdout_stderr.log` or `streams.log`.
- Deliberately add one flake8 and one clang-format defect and verify both package tests and
  `scripts/lint.sh --all` catch them.
- Verify the audit handles pure interface, CMake-only, Python-only, mixed Python/CMake, and vendor
  packages.
- Verify `BUILD_TESTING=OFF` does not require lint dependencies.

## Acceptance criteria

- [ ] Every first-party package's lint declarations and registrations agree.
- [ ] Gazebo and robot MCP lint checks are collected by `colcon test`.
- [ ] The 26 current ament_flake8 findings are resolved or explicitly baselined with rationale.
- [ ] Ruff-versus-ament differences are documented and reproducible.
- [ ] Vendor formatting and lint policy is explicit and tested.
- [ ] `scripts/lint.sh --all` reproduces the required package lint gates.
- [ ] A fast audit prevents future declaration-without-registration gaps.
