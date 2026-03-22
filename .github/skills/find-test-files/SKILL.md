---
name: find-test-files
description: 'Locate test files in the workspace by searching for common test file patterns and conventions. Use when asked to "find tests", "locate test files", "list all tests", "discover test suites", or when working with TDD workflows, test analysis, or test coverage tasks. Supports Python, C++, TypeScript, JavaScript, and ROS 2 test patterns.'
---

# Find Test Files

Locate test files across the codebase using pattern matching and common test file naming conventions. This skill provides the functionality equivalent to a `findTestFiles` tool by leveraging filesystem search capabilities.

## When to Use This Skill

Use this skill when you need to:

- Find all test files in a project
- Locate tests for a specific package or module
- Identify test suites for test-driven development (TDD) workflows
- Analyze test coverage by discovering what test files exist
- Prepare for test execution by building a list of test files
- Search for tests matching specific patterns or technologies

## Prerequisites

- Workspace root directory available

## Supported Test File Patterns

### Python Test Patterns

- `test_*.py` - Pytest and unittest standard
- `*_test.py` - Alternative Python test convention
- `tests/**/*.py` - Tests directory structure
- Files containing `@pytest` or `class Test`

### C++ Test Patterns

- `*_test.cpp`, `*_test.cc` - Google Test convention
- `test_*.cpp`, `test_*.cc` - Alternative C++ pattern
- `Test*.cpp`, `Test*.cc` - CamelCase test naming
- `*_unittest.cpp` - Unit test naming
- `test/**/*.cpp` - Tests directory structure
- Files containing `TEST(`, `TEST_F(`, `TYPED_TEST(`

### JavaScript/TypeScript Test Patterns

- `*.test.js`, `*.test.ts` - Jest/Vitest convention
- `*.spec.js`, `*.spec.ts` - Jasmine/Mocha convention
- `__tests__/**/*.js`, `__tests__/**/*.ts` - Jest standard
- Files containing `describe(`, `it(`, `test(`

### ROS 2 Test Patterns

- `test/**/*.cpp` - ROS 2 C++ tests
- `test/**/*.py` - ROS 2 Python tests
- `*_test.cpp`, `test_*.py` - ROS package tests
- Files with `ament_add_*test` in parent CMakeLists.txt

## Core Workflows

### Workflow 1: Find All Tests in Workspace

**Use the `search` tool with file pattern matching:**

```
#search pattern:"test_*.py OR *_test.py OR *.test.ts OR *_test.cpp"
```

Or use `search/codebase` to scan for test directories:

```
#codebase - Look for directories named: test, tests, __tests__
```

**Expected output:** List of all matching test file paths

### Workflow 2: Find Tests for Specific Package

**For a ROS 2 package named `drqp_serial`:**

```
#search path:"packages/runtime/drqp_serial/test/**"
```

**For a Python module:**

```
#search pattern:"test_<module_name>.py OR <module_name>_test.py"
```

**Expected output:** Test files specific to that package/module

### Workflow 3: Find Tests by Technology

**Python tests only:**

```
#search pattern:"test_*.py OR *_test.py" type:file
```

**C++ tests only:**

```
#search pattern:"*_test.cpp OR *_test.cc OR test_*.cpp OR Test*.cpp" type:file
```

**JavaScript/TypeScript tests only:**

```
#search pattern:"*.test.js OR *.test.ts OR *.spec.js OR *.spec.ts" type:file
```

**Expected output:** Filtered list by language/framework

### Workflow 4: Find Tests with Content Matching

**Find tests that test a specific function or class:**

```
#search content:"TEST(MyClass" OR content:"test_my_function"
```

**Find integration tests:**

```
#search content:"integration" AND pattern:"*test*.py OR *test*.cpp"
```

**Expected output:** Test files containing specific test cases

## Implementation Guidelines

### Using Available Tools

Since `findTestFiles` is not available, use these alternative approaches:

1. **File Pattern Search** - Use `search` or `search/codebase` with glob patterns
2. **Directory Traversal** - Search in known test directories (`test/`, `tests/`, `__tests__/`)
3. **Content-Based Discovery** - Search for test framework imports/keywords
4. **Build System Integration** - Parse CMakeLists.txt, package.json, or setup.py for test definitions

### Combining Multiple Patterns

To find all tests across technologies:

```markdown
Search for test files using multiple patterns:
1. Python: test_*.py, *_test.py
2. C++: *_test.cpp, *_test.cc, test_*.cpp, Test*.cpp
3. TypeScript/JavaScript: *.test.ts, *.spec.ts
4. ROS 2: test/**/*.{cpp,py}

Combine results and remove duplicates.
```

### Language-Specific Conventions

#### Python (pytest/unittest)

- Primary: `test_*.py` in any directory
- Secondary: Files in `tests/` or `test/` directories
- Content markers: `import pytest`, `import unittest`, `def test_`, `class Test`

#### C++ (Google Test/Catch2)

- Primary: `*_test.cpp` in test directories
- Secondary: Files with `TEST(`, `TEST_F(`, `ASSERT_`, `EXPECT_`
- Build integration: Check CMakeLists.txt for `ament_add_gtest`, `add_test`

#### JavaScript/TypeScript (Jest/Vitest/Mocha)

- Primary: `*.test.{js,ts,jsx,tsx}`, `*.spec.{js,ts,jsx,tsx}`
- Secondary: `__tests__/**/*` directories
- Content markers: `describe(`, `it(`, `test(`, `expect(`

## Example Queries

### Example 1: TDD Red Phase - Find Existing Tests

**User request:** "Show me all existing tests for the serial driver"

**Agent action:**
```
#search path:"packages/runtime/drqp_serial/test/**"
```

**Response format:**
```
Found 3 test files for drqp_serial:
1. packages/runtime/drqp_serial/test/test_serial_connection.cpp
2. packages/runtime/drqp_serial/test/test_serial_protocol.cpp
3. packages/runtime/drqp_serial/test/integration/test_serial_hardware.cpp
```

### Example 2: Test Coverage Analysis

**User request:** "List all Python test files to check coverage"

**Agent action:**
```
#search pattern:"test_*.py" type:file
#search pattern:"*_test.py" type:file
```

**Response format:**
```
Python test files found (15 total):
Runtime packages:
  - packages/runtime/drqp_brain/test/test_state_machine.py
  - packages/runtime/drqp_control/test/test_controller.py
  ...
Simulation packages:
  - packages/simulation/drqp_gazebo/test/test_spawn.py
```

### Example 3: Find Tests for Refactoring

**User request:** "Find all tests that reference the old ConnectionManager class"

**Agent action:**
```
#search content:"ConnectionManager" AND pattern:"*test*.{py,cpp}"
```

**Response format:**
```
Tests referencing ConnectionManager (4 files):
1. test/test_serial_connection.cpp:15 - TEST(ConnectionManager, InitTest)
2. test/test_protocol.py:23 - def test_connection_manager_setup():
3. integration/test_hardware.cpp:45 - Uses ConnectionManager mock
4. test/test_reconnect.cpp:10 - Tests ConnectionManager reconnection logic
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| No tests found | Check if test directory exists; verify naming conventions match project standards |
| Too many false positives | Add content-based filtering (search for `TEST(`, `def test_`, etc.) |
| Missing nested tests | Use recursive patterns: `test/**/*.py`, `**/test/*.cpp` |
| Tests in non-standard locations | Search by content markers instead of filename patterns |
| Performance issues with large codebases | Narrow search to specific directories or packages |

## Common Patterns

### Pattern: Comprehensive Test Discovery

For projects with multiple test frameworks:

```bash
# Search all common test patterns
find . -type f \( \
  -name "test_*.py" -o \
  -name "*_test.py" -o \
  -name "*_test.cpp" -o \
  -name "*_test.cc" -o \
  -name "test_*.cpp" -o \
  -name "Test*.cpp" -o \
  -name "*.test.ts" -o \
  -name "*.test.js" -o \
  -name "*.spec.ts" -o \
  -name "*.spec.js" \
\) | grep -v node_modules | grep -v build | sort
```

### Pattern: ROS 2 Package Test Discovery

For ROS 2 workspaces with colcon:

```bash
# Find tests organized by package
find packages -type d -name "test" -exec sh -c 'echo "Package: $(basename $(dirname $1))"; find "$1" -type f \( -name "*.py" -o -name "*.cpp" \)' _ {} \;
```

### Pattern: Group Tests by Type

Organize discovered tests by type:

```
Unit Tests:
  - Files in test/unit/
  - Names containing "_unit_test"

Integration Tests:
  - Files in test/integration/
  - Names containing "_integration_test"

End-to-End Tests:
  - Files in test/e2e/
  - Names containing "_e2e_test"
```

## Integration with TDD Workflow

### Red Phase (Write Failing Tests)

1. Use this skill to find existing tests
2. Identify where new test should be placed
3. Follow naming convention of nearby tests

### Green Phase (Make Tests Pass)

1. Quickly locate the failing test file
2. Run specific test file
3. Iterate on implementation

### Refactor Phase (Improve Code Quality)

1. Find all tests that touch refactored code
2. Ensure all tests still pass
3. Update test names if refactoring changed behavior

## Limitations

- Cannot execute tests (use `execute/runTests` for that)
- May miss tests in non-standard locations
- Requires consistent naming conventions
- Large codebases may need scoped searches
- Build-generated test files may not be visible in source search

## Additional Resources

- ROS 2 Testing Guide: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html
- pytest Documentation: https://docs.pytest.org/
- Google Test Primer: https://google.github.io/googletest/primer.html
- Jest Testing Framework: https://jestjs.io/docs/getting-started
