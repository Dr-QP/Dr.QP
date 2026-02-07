# Agents Guidelines

## General Engineering Principles

For comprehensive engineering standards and best practices, refer to [shared engineering guidelines](/.github/instructions/engineering.instructions.md).

**Quick reference**:

- Prioritize readability and clarity
- Apply Clean Code and SOLID principles
- Handle edge cases and exceptions explicitly
- Use consistent, idiomatic patterns
- Follow language-specific conventions

## Available Agents

This workspace provides specialized agents for different development tasks. You can request a specific agent by name, or choose based on your task:

### Planning & Analysis Agents

- **[Task Planner](/.github/agents/task-planner.agent.md)** - Create actionable implementation plans
- **[Task Researcher](/.github/agents/task-researcher.agent.md)** - Comprehensive project analysis and research
- **[Technical Spike Researcher](/.github/agents/technical-spike-researcher.agent.md)** - Systematically research and validate technical spike documents
- **[Issue Refiner](/.github/agents/issue-refiner.agent.md)** - Refine requirements with acceptance criteria, technical considerations, and edge cases

### Development Agents

- **[Code Alchemist](/.github/agents/code-alchemist.agent.md)** - Transform code with Clean Code principles and SOLID design
- **[C++ Expert](/.github/agents/cpp-expert.agent.md)** - Expert C++ software engineering guidance using modern C++ practices
- **[React Expert](/.github/agents/react-expert.agent.md)** - Expert React frontend engineer with modern hooks and performance optimization
- **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** - Principal-level software engineering guidance with focus on excellence and pragmatic implementation

### Testing & Quality Agents

- **[TDD Red](/.github/agents/tdd-red.agent.md)** - Write failing tests that describe desired behavior before implementation
- **[TDD Green](/.github/agents/tdd-green.agent.md)** - Implement minimal code to satisfy requirements and make tests pass
- **[TDD Refactor](/.github/agents/tdd-refactor.agent.md)** - Improve code quality while maintaining tests and compliance

### Debugging & Review Agents

- **[Debugger](/.github/agents/debug.agent.md)** - Debug your application to find and fix bugs
- **[Security Sentinel](/.github/agents/security-sentinel.agent.md)** - Review code for security issues and vulnerabilities
- **[Tech Debt Remediator](/.github/agents/tech-debt-remediator.agent.md)** - Generate technical debt remediation plans

### Support Agents

- **[Prompt Builder](/.github/agents/prompt-builder.agent.md)** - Expert prompt engineering and validation system
- **[Mentor](/.github/agents/mentor.agent.md)** - Challenge assumptions and mentor engineers through Socratic questioning
- **[Technical Content Evaluator](/.github/agents/technical-content-evaluator.agent.md)** - Review technical materials for accuracy and pedagogical excellence
- **[Custom Agent Foundry](/.github/agents/custom-agent-foundry.agent.md)** - Design and create VS Code custom agents
- **[Deep Thinker](/.github/agents/deep-thinker.agent.md)** - Advanced reasoning with creative freedom for complex problems

### When in Doubt

Consult with the **Principal Engineer** agent for senior-level guidance on architecture, design decisions, and implementation strategies.

## Available Skills

This workspace provides specialized Agent Skills that enhance your capabilities:

- **[ros2-workspace-build](/.github/skills/ros2-workspace-build/SKILL.md)**: Build ROS 2 packages with colcon (incremental builds, debug symbols, coverage)
- **[ros2-workspace-testing](/.github/skills/ros2-workspace-testing/SKILL.md)**: Test packages and generate coverage reports
- **[ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md)**: Manage workspace dependencies via rosdep and pip
- **[ros2-environment-setup](/.github/skills/ros2-environment-setup/SKILL.md)**: Initialize and configure the development environment
- **[ros2-launch-management](/.github/skills/ros2-launch-management/SKILL.md)**: Create, configure, and debug ROS 2 launch files with parameter passing and composition
- **[ros2-parameter-tuning](/.github/skills/ros2-parameter-tuning/SKILL.md)**: Configure and tune node parameters using YAML files and runtime commands
- **[ros2-diagnostics](/.github/skills/ros2-diagnostics/SKILL.md)**: Debug and troubleshoot ROS 2 systems using introspection and diagnostic tools
- **[ros2-lifecycle-management](/.github/skills/ros2-lifecycle-management/SKILL.md)**: Manage lifecycle node state transitions and coordinate system startup
- **[code-review-standards](/.github/skills/code-review-standards/SKILL.md)**: Write PR descriptions and conduct code reviews
- **[find-test-files](/.github/skills/find-test-files/SKILL.md)**: Identify and summarize relevant test files for a given change or component

## Available Prompts

This workspace provides reusable prompt files for streamlining common development workflows. Prompts are pre-configured templates that guide GitHub Copilot through multi-step tasks.

### Package and Project Setup

- **[create-ros2-package](/.github/prompts/create-ros2-package.prompt.md)**: Create new ROS 2 package with proper structure, build files, and test scaffolding (C++, Python, or mixed)
- **[add-ci-workflow](/.github/prompts/add-ci-workflow.prompt.md)**: Generate GitHub Actions workflows for build, test, coverage, or documentation automation

### Development Patterns

- **[implement-publisher-subscriber](/.github/prompts/implement-publisher-subscriber.prompt.md)**: Create ROS 2 publisher/subscriber nodes with proper QoS configuration and message handling
- **[create-state-machine](/.github/prompts/create-state-machine.prompt.md)**: Generate state machine implementation with states, transitions, events, and lifecycle management

### Testing and Quality

- **[add-test-file](/.github/prompts/add-test-file.prompt.md)**: Add unit or integration test file with fixtures, test cases, and build integration
- **[generate-pr-description](/.github/prompts/generate-pr-description.prompt.md)**: Create comprehensive pull request description following code-review-standards

### How to Use Prompts

In GitHub Copilot Chat:
1. Type `/` to see available prompts
2. Select a prompt or type the prompt name
3. Provide required inputs when prompted
4. Review and apply generated output

Example:
```
/create-ros2-package drqp_sensors --type cpp --deps rclcpp,sensor_msgs
```

For detailed documentation, see [Prompts README](/.github/prompts/README.md).

## ROS 2 Development Environment

This is a ROS 2 Jazzy workspace using colcon build system for C++ and Python packages.

For detailed workflow guidance, use the relevant **Agent Skill** for your task:

- **Environment Setup**: Use [ros2-environment-setup](/.github/skills/ros2-environment-setup/SKILL.md)
- **Building**: Use [ros2-workspace-build](/.github/skills/ros2-workspace-build/SKILL.md)
- **Testing**: Use [ros2-workspace-testing](/.github/skills/ros2-workspace-testing/SKILL.md)
- **Dependencies**: Use [ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md)

## Package Structure

The workspace contains the following package categories:

- **runtime/**: Core runtime packages
  - `drqp_interfaces`: ROS message/service definitions
  - `drqp_serial`: Serial communication driver
  - `drqp_a1_16_driver`: A1-16 servo controller driver
  - `drqp_control`: Robot control nodes
  - `drqp_brain`: High-level robot behavior and state machine
  - `drqp_lint_common`: Common linting configurations
  - `drqp_rapidjson`: RapidJSON vendor package

- **simulation/**: Simulation packages
  - `drqp_gazebo`: Gazebo simulation integration

- **cmake/**: CMake utilities and macros

### CI/CD Pipeline

The GitHub Actions workflow (`.github/workflows/ci.yml`) runs:

1. **Build development Docker image** (`jazzy-ros-desktop`)
2. **Build ROS workspace** with coverage enabled
3. **Run all tests** with coverage collection
4. **Process coverage reports** and upload to Codecov
5. **Test devcontainer** to ensure it works
6. **Build deployment image** for production use

**Key CI commands:**

```bash
# Install dependencies
./scripts/ros-dep.sh

# Build with coverage
python3 -m colcon build --mixin coverage-pytest ninja rel-with-deb-info \
  --event-handlers=console_cohesion+ --symlink-install \
  --cmake-args -D DRQP_ENABLE_COVERAGE=ON

# Run tests
python3 -m colcon test --mixin coverage-pytest \
  --event-handlers=console_cohesion+ --return-code-on-test-failure

# Process coverage
./packages/cmake/llvm-cov-export-all.py ./build
```

### Dependency Management

For comprehensive dependency management guidance (C++, Python, rosdep, pip), see the [ros2-dependency-management](/.github/skills/ros2-dependency-management/SKILL.md) skill.

### Cleaning Build Artifacts

```bash
# Clean build, install, and log directories
./scripts/clean.fish
```

## Best Practices for Agents

1. **Always source setup.bash** before any build or test operation
2. **Use incremental builds** (`--packages-up-to <pkg>`) during development
3. **Test specific packages** (`--packages-select <pkg>`) for rapid iteration
4. **Use devcontainer** when running as a remote agent
5. **Only run full builds/tests** when explicitly requested or necessary
6. **Collect test output** from `build/<package_name>/test_results/` for analysis
7. **Check build logs** in `log/latest_build/` if builds fail
8. **Use `--symlink-install`** for faster Python development iteration
9. **Enable coverage** with `--mixin coverage-pytest` when testing
10. **Re-run failed tests** with `--packages-select-test-failures` to save time

## Code Review Standards

For comprehensive code review guidance and PR description standards, see the [code-review-standards](/.github/skills/code-review-standards/SKILL.md) skill.
