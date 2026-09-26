---
type: codebase
description: A drop-in replacement for ament_lint_common that selects the project's linter set — clang-format instead of uncrustify.
source: packages/runtime/drqp_lint_common
source_digest: sha256:551f5e68c73254d35ca78041d73039013619c922bd0d68696f5de8d7aff1d2b4
verified:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
stale_after: 2026-12-25
generated:
  by: claude-code/opus-5.5
  at: 2026-09-26T20:42:30Z
sources:
- id: code
  resource: packages/runtime/drqp_lint_common
---

# drqp_lint_common

"A set of linters for Dr.QP project to be used instead of ament_lint_common." It
is a manifest-only CMake package, modelled on the Jazzy `ament_lint_common`
package.

## Public surface

- Exec dependencies re-exported to dependants:
  - `ament_cmake_clang_format`
  - `ament_cmake_copyright`
  - `ament_cmake_cppcheck`
  - `ament_cmake_cpplint`
  - `ament_cmake_flake8`
  - `ament_cmake_lint_cmake`
  - `ament_cmake_pep257`
  - `ament_cmake_xmllint`

## How it works

A package lists `drqp_lint_common` as a `test_depend` and calls
`ament_lint_auto_find_test_dependencies()`. `ament_lint_auto` then registers a
test for each linter exported here. The users are `drqp_serial`,
`drqp_a1_16_driver`, `drqp_control`, `drqp_interfaces`, `drqp_joy`,
`drqp_moveit`, `drqp_gazebo`, and `sdl3_vendor`.

## Invariants & gotchas

- `ament_cmake_uncrustify` is left out on purpose: "this check conflicts with
  clangd formatting". `ament_cmake_clang_tidy` is commented out.
- The Python-only packages do not use it. They carry their own `test_flake8.py`,
  `test_pep257.py`, and `test_copyright.py`.

## Key references

Verified anchor points (line numbers as of 2026-09-26):

- `packages/runtime/drqp_lint_common/package.xml` — linter `exec_depend` list
  and the uncrustify note
- `packages/runtime/drqp_lint_common/CMakeLists.txt` —
  `ament_export_dependencies` of the exec depends
- `packages/runtime/drqp_serial/CMakeLists.txt:71` —
  `ament_lint_auto_find_test_dependencies` consumer
