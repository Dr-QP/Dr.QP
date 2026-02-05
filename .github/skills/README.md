# Skills

Agent Skills for the Dr.QP project. These skills provide specialized capabilities that enhance AI agent functionality.

## Available Skills

### find-test-files

**Description:** Locate test files in the workspace by searching for common test file patterns and conventions.

**When to use:** Use when asked to "find tests", "locate test files", "list all tests", "discover test suites", or when working with TDD workflows, test analysis, or test coverage tasks.

**Supports:**
- Python test patterns (pytest, unittest)
- C++ test patterns (Google Test, Catch2)  
- TypeScript/JavaScript test patterns (Jest, Vitest, Mocha, Jasmine)
- ROS 2 test patterns

**Key features:**
- Comprehensive test file pattern matching
- Language-specific test discovery strategies
- Integration with TDD workflows (Red, Green, Refactor phases)
- Content-based test file discovery
- Package/module-scoped test searches

**File:** [find-test-files/SKILL.md](find-test-files/SKILL.md)

### ros2-workspace-build

**Description:** Build ROS 2 workspaces using `colcon` or similar tooling. Use this skill to configure, build, and rebuild ROS 2 packages in a workspace, including handling common build profiles and options.

**When to use:** Use when asked to "build the ROS 2 workspace", "run colcon build", "fix build errors", "set up a ROS 2 build pipeline", or when working on CI workflows that compile ROS 2 packages.

**Supports:**
- `colcon build` workflows
- Debug/release build profiles
- Incremental and clean builds
- Workspace overlay and underlay patterns

**Key features:**
- Standardized ROS 2 build commands and options
- Guidance for resolving common ROS 2 build failures
- Recommendations for CI build steps

**File:** [ros2-workspace-build/SKILL.md](ros2-workspace-build/SKILL.md)

### ros2-workspace-testing

**Description:** Run and manage tests in a ROS 2 workspace. Use this skill to execute `colcon test`, filter tests, interpret test results, and integrate ROS 2 tests into CI pipelines.

**When to use:** Use when asked to "run ROS 2 tests", "execute colcon test", "debug ROS 2 test failures", "list failing tests", or when setting up test stages for ROS 2 in CI.

**Supports:**
- `colcon test` workflows
- Selecting or filtering specific test packages
- Test result and log inspection
- Integration with common ROS 2 test frameworks

**Key features:**
- Standardized test command patterns
- Guidance for troubleshooting failing ROS 2 tests
- Suggestions for improving ROS 2 test coverage

**File:** [ros2-workspace-testing/SKILL.md](ros2-workspace-testing/SKILL.md)

### ros2-environment-setup

**Description:** Configure and validate ROS 2 development environments. Use this skill to source setup files, manage overlays, and ensure that environment variables are correctly configured for ROS 2 workspaces.

**When to use:** Use when asked to "source the ROS 2 workspace", "set up the ROS 2 environment", "configure overlays/underlays", "fix ROS_DOMAIN_ID or RMW issues", or when onboarding a new ROS 2 development machine.

**Supports:**
- Sourcing global and workspace-specific `setup.*` scripts
- Managing multiple ROS 2 distributions and overlays
- Verifying core environment variables and paths

**Key features:**
- Step-by-step ROS 2 environment setup workflows
- Checks for common misconfigurations
- Guidance for cross-shell and cross-platform setup patterns

**File:** [ros2-environment-setup/SKILL.md](ros2-environment-setup/SKILL.md)

### ros2-dependency-management

**Description:** Analyze and manage dependencies in ROS 2 workspaces. Use this skill to work with `package.xml`, `rosdep`, and other tools for installing, updating, and validating ROS 2 dependencies.

**When to use:** Use when asked to "install ROS 2 dependencies", "run rosdep", "fix missing ROS 2 packages", "analyze package.xml dependencies", or when preparing a workspace for build or CI on a new machine.

**Supports:**
- `rosdep` resolution and installation workflows
- Dependency analysis from `package.xml` and `CMakeLists.txt`
- Differentiating build, run, and test dependencies

**Key features:**
- Standardized dependency installation steps
- Guidance for resolving missing or conflicting ROS 2 dependencies
- Recommendations for dependency management in CI and container images

**File:** [ros2-dependency-management/SKILL.md](ros2-dependency-management/SKILL.md)

### code-review-standards

**Description:** Provide project-specific code review standards and checklists. Use this skill when performing or responding to code reviews to ensure consistency, quality, and adherence to the Dr.QP project's guidelines.

**When to use:** Use when asked to "review this code", "apply our code review standards", "check for style and quality issues", "prepare a PR for review", or when drafting review feedback.

**Supports:**
- Consistent review criteria across languages and components
- Style, readability, and maintainability checks
- Test coverage and documentation expectations

**Key features:**
- Structured review checklists and heuristics
- Suggestions for actionable, constructive feedback
- Alignment with project-specific patterns and conventions

**File:** [code-review-standards/SKILL.md](code-review-standards/SKILL.md)
## How to Use Skills

Skills are automatically discovered by GitHub Copilot when they are placed in the `.github/skills/` directory. The agent will load the skill instructions when your request matches the skill's description.

## Creating New Skills

To create a new skill:

1. Create a new directory under `.github/skills/` with a lowercase, hyphenated name
2. Create a `SKILL.md` file with proper YAML frontmatter:
   ```yaml
   ---
   name: my-skill-name
   description: 'Clear description of what the skill does and when to use it.'
   ---
   ```
3. Add detailed instructions in the body of `SKILL.md`
4. Optionally add bundled assets (scripts, references, templates) in subdirectories

For more information, see the [Agent Skills specification](https://agentskills.io/specification).
