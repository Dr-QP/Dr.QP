# Agents Guidelines

NEVER use "$TMPDIR" env variable.
ALWAYS use "./.tmp" (relative to the repo root) for temporary files; create it if it does not exist.

## General Engineering Principles

For comprehensive engineering standards and best practices, refer to [shared engineering guidelines](/.github/instructions/engineering.instructions.md).

**Quick reference**:

- Prioritize readability and clarity
- Apply Clean Code and SOLID principles
- Handle edge cases and exceptions explicitly
- Use consistent, idiomatic patterns
- Follow language-specific conventions

## Cursor Compatibility

This repository keeps GitHub Copilot customizations under `.github/`. Cursor
reads from `.cursor/`, so we link the agent, skill, and instruction catalogs there:

- **Agents**: `.cursor/agents/` (symlink to `.github/agents/`)
- **Skills**: `.cursor/skills/` (symlink to `.github/skills/`)
- **Instructions**: `.cursor/rules/` (symlink to `.github/instructions/`)

Update the `.github` sources; the symlinks pick up changes automatically.

## Codex Compatibility

This repository keeps GitHub Copilot customizations under `.github/`. Codex
reads from `.codex/`, so we link the supported agentic catalogs there:

- **Agents**: `.codex/agents/` (symlink to `.github/agents/`)
- **Skills**: `.codex/skills/` (symlink to `.github/skills/`)
- **Instructions**: `.codex/instructions/` (symlink to `.github/instructions/`)
- **Prompts**: `.codex/prompts/` (symlink to `.github/prompts/`)

Update the `.github` sources; the symlinks pick up changes automatically.

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
- **[Feedback Handler](/.github/agents/feedback-handler.agent.md)** - Autonomously address PR feedback, CI failures, CodeQL findings, and coverage gaps

### Support Agents

- **[Prompt Builder](/.github/agents/prompt-builder.agent.md)** - Expert prompt engineering and validation system
- **[Mentor](/.github/agents/mentor.agent.md)** - Challenge assumptions and mentor engineers through Socratic questioning
- **[Technical Content Evaluator](/.github/agents/technical-content-evaluator.agent.md)** - Review technical materials for accuracy and pedagogical excellence
- **[Custom Agent Foundry](/.github/agents/custom-agent-foundry.agent.md)** - Design and create VS Code custom agents
- **[Deep Thinker](/.github/agents/deep-thinker.agent.md)** - Advanced reasoning with creative freedom for complex problems

### When in Doubt

Consult with the **Principal Engineer** agent for senior-level guidance on architecture, design decisions, and implementation strategies.

## Remote Agent Guidelines

### Addressing PR Feedback

**For remote/cloud agents**: When tasked with addressing PR review comments, CI failures, security findings, or test coverage gaps, you MUST use the **[Feedback Handler](/.github/agents/feedback-handler.agent.md)** agent.

The Feedback Handler provides:

- Systematic resolution of all feedback types
- Intent classification to avoid misinterpreting comments
- TDD methodology orchestration via principal-engineer
- Safety controls to prevent risky changes
- Comprehensive tracking and documentation

See [PR Feedback Instructions](/.github/instructions/pr-feedback.instructions.md) for detailed guidance on when and how to invoke the Feedback Handler agent.

## Available Skills

This workspace uses skill metadata in each `SKILL.md` file as the single source of truth for skill discovery and routing intent.

- Browse skill folders under [`.github/skills/`](/.github/skills/)
- See [`.github/skills/README.md`](/.github/skills/README.md) for validation commands and conventions

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

## Cursor Cloud specific instructions

### Development Container

All build, test, and lint commands run inside the devcontainer defined in `.devcontainer/devcontainer.json`. The cloud VM does not have ROS 2 installed natively.

The update script starts Docker and brings up the devcontainer using:

```bash
devcontainer up --workspace-folder /workspace \
  --mount "type=bind,source=/var/run/docker.sock,target=/var/run/docker.sock"
```

The host Docker socket mount lets `scripts/devcontainer-start-docker.sh` (`postStartCommand`) detect that Docker is already accessible and exit cleanly, avoiding a redundant `dockerd` startup inside the container. The `devcontainer up` command runs all lifecycle hooks defined in `devcontainer.json`: `postCreateCommand` (directory ownership), `postStartCommand` (Docker check), and `postAttachCommand` (rosdep install, Python venv creation, xpra startup).

### Running Commands

Use `devcontainer exec` to run commands inside the container:

```bash
devcontainer exec --workspace-folder /workspace bash -c "<command>"
```

Before build or ROS 2 commands, source the setup script inside the exec:

```bash
devcontainer exec --workspace-folder /workspace bash -c "
  export ROS_DISTRO=jazzy CC=clang CXX=clang++ CMAKE_EXPORT_COMPILE_COMMANDS=1
  source scripts/setup.bash
  <ros2 commands here>
"
```

For test runs that need the production venv (with `python-statemachine`), use `source scripts/setup.bash --update-venv` **after a successful `colcon build`**. The `--update-venv` flag scans `build/` and `install/` for `requires.txt` files. Inside the devcontainer these directories exist as Docker volume mount points, but they will be empty until the first build — so `find` may return nothing (harmless). Outside the devcontainer the directories may not exist at all, producing noisy errors. Build, test, and lint commands are documented in the CI/CD Pipeline section above.

### Gotchas

- The colcon mixin config lives at `/root/.colcon/` inside the image and is pre-configured.
- `--symlink-install` is required for Python coverage collection and hot-reloading.
- Some packages emit CMake warnings about unused `DRQP_ENABLE_COVERAGE` — these are benign.
- The `drqp_gazebo` simulation tests run headless and take ~20 seconds.
- The production venv (`.venv-prod`) is separate from the dev venv (`.venv`). The `setup.bash --update-venv` flag populates `.venv-prod` from `requires.txt` files in `build/` and `install/` directories.
- The cloud VM's host Docker daemon must be running before `devcontainer up`. The update script starts it with `sudo dockerd &>/tmp/dockerd.log &`. The host daemon uses `fuse-overlayfs` as configured in `/etc/docker/daemon.json` because the cloud VM runs inside a Firecracker VM where the default `overlay2` driver is not supported. This is unrelated to Docker inside the devcontainer (which is handled by `scripts/devcontainer-start-docker.sh`).
