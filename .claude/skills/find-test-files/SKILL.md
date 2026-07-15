---
name: find-test-files
description: Locate Python, C++, and ROS 2 test files in this workspace with rg filename and content searches. Use when asked to find tests, list test suites, locate tests for a package or module, or discover tests before TDD, coverage, or refactoring work.
---

# Find Test Files

Use `rg` to discover relevant tests without relying on unavailable editor
pseudo-tools. Start with the smallest path that can answer the request; search
generic JavaScript or TypeScript patterns only when the user explicitly asks
about a JavaScript/TypeScript component.

## Find tests by package or language

For a named ROS package anywhere under `packages/`, list conventional test
files first:

```bash
package_name=drqp_joy  # Replace with the requested package name.
rg --files packages \
  -g "**/${package_name}/test/test_*.py" \
  -g "**/${package_name}/test/*_test.py" \
  -g "**/${package_name}/test/test_*.cpp" \
  -g "**/${package_name}/test/*_test.cpp" \
  -g "**/${package_name}/test/Test*.cpp" \
  -g "**/${package_name}/test/test_*.cc" \
  -g "**/${package_name}/test/*_test.cc" \
  -g "**/${package_name}/test/Test*.cc"
```

For a workspace-wide inventory, search source packages and exclude generated
outputs:

```bash
rg --files packages \
  -g 'test_*.py' -g '*_test.py' \
  -g 'test_*.cpp' -g '*_test.cpp' -g 'Test*.cpp' \
  -g 'test_*.cc' -g '*_test.cc' -g 'Test*.cc' \
  -g '!build/**' -g '!install/**' -g '!log/**' | sort
```

Narrow the globs to `*.py` or C++ patterns when the request names a language.
Also inspect package test registration when filenames alone are insufficient:

```bash
rg -n 'ament_add_(gtest|gmock|pytest_test)|add_catch2_unit_test\(|add_test\(' \
  -g 'CMakeLists.txt' packages
```

## Find tests that exercise a symbol or behavior

Use a literal or regular-expression search with test-file globs. For example:

```bash
rg -l 'ConnectionManager' packages \
  -g 'test_*.py' -g '*_test.py' \
  -g 'test_*.cpp' -g '*_test.cpp' -g 'Test*.cpp' \
  -g 'test_*.cc' -g '*_test.cc' -g 'Test*.cc'
rg -n 'def test_|class Test|TEST(_F|_P)?\(' packages/<area>/<package_name>/test
```

Report the paths, and include matching line numbers only when the caller needs
the relevant test case or a refactoring impact assessment. Do not infer test
coverage solely from matching filenames; inspect the file when behavior matters.

## Optional JavaScript/TypeScript scope

For an explicitly requested JS/TS package, adapt the same command to that
package rather than scanning the ROS workspace indiscriminately:

```bash
rg --files <package_path> \
  -g '*.test.js' -g '*.test.ts' -g '*.test.jsx' -g '*.test.tsx' \
  -g '*.spec.js' -g '*.spec.ts' -g '*.spec.jsx' -g '*.spec.tsx'
```

Use the following command only to find tests that use nonstandard names:

```bash
rg -l 'describe\(|it\(|test\(' <package_path>
```

## Hand off after discovery

Use [add-test-file-python](../add-test-file-python/) or
[add-test-file-cpp](../add-test-file-cpp/) to add a missing test. Use
[ros2-workspace-testing](../ros2-workspace-testing/) to run the selected
package's tests; this skill discovers files and does not execute them.
