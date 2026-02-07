# Prompt Files

This directory contains reusable prompt files for GitHub Copilot Chat that streamline common development workflows in the ROS 2 workspace.

## What Are Prompt Files?

Prompt files (`.prompt.md`) are pre-configured, executable templates that guide GitHub Copilot through multi-step workflows. They reduce cognitive load, ensure consistency, and automate repetitive tasks by providing:

- Clear input specifications
- Step-by-step instructions
- Validation and quality assurance
- Output expectations
- Related resource links

## Available Prompts

### Package and Project Setup

#### 1. [create-ros2-package](./create-ros2-package.prompt.md)

**Purpose**: Create a new ROS 2 package with proper structure, build files, and test scaffolding.

**When to use**:
- Starting a new ROS 2 package
- Need complete package structure (C++, Python, or mixed)
- Want proper CMakeLists.txt/setup.py configuration
- Setting up test infrastructure from the start

**Example**:
```
/create-ros2-package drqp_my_package --type cpp --deps rclcpp,std_msgs
```

**Inputs**:
- Package name (required)
- Package type: `cpp`, `python`, or `mixed` (required)
- Dependencies (optional)
- Package path (default: `packages/runtime`)

**Outputs**:
- Complete package directory structure
- package.xml with dependencies
- CMakeLists.txt or setup.py
- Test scaffolding
- README.md

---

### Testing

#### 2. [add-test-file](./add-test-file.prompt.md)

**Purpose**: Add unit or integration test file with fixtures, test cases, and build integration.

**When to use**:
- Adding tests for existing source files
- Following TDD workflow
- Need proper test fixtures and scaffolding
- Want tests that match project conventions

**Example**:
```
/add-test-file src/MyClass.cpp --type unit
```

**Inputs**:
- Source file to test (required, can use current file)
- Test type: `unit` or `integration` (required)
- Package name (auto-detected or ask)

**Outputs**:
- Test file with GTest/pytest structure
- Test fixtures and multiple test cases
- CMakeLists.txt updated (C++) or auto-discovery (Python)
- Test dependencies verified in package.xml

---

### ROS 2 Patterns

#### 3. [implement-publisher-subscriber](./implement-publisher-subscriber.prompt.md)

**Purpose**: Create ROS 2 publisher and subscriber nodes with proper initialization and QoS configuration.

**When to use**:
- Implementing topic-based communication
- Need publisher/subscriber pair
- Want proper QoS profiles
- Creating example or template nodes

**Example**:
```
/implement-publisher-subscriber sensor_msgs/msg/Image /camera/image --qos sensor_data
```

**Inputs**:
- Message type (e.g., `std_msgs/msg/String`, `sensor_msgs/msg/Image`)
- Topic name (e.g., `/my_topic`, `/robot/state`)
- QoS profile (optional): `default`, `reliable`, `best_effort`, `sensor_data`
- Publish rate (optional, for publisher)
- Language: `cpp` or `python` (optional)

**Outputs**:
- Publisher node with timer-based publishing
- Subscriber node with message callback
- QoS configuration
- Build system updated
- Launch file (optional)

---

#### 4. [create-state-machine](./create-state-machine.prompt.md)

**Purpose**: Create state machine implementation with states, transitions, events, and lifecycle management.

**When to use**:
- Implementing robot behavior control
- Need finite state machine for system modes
- Building reactive control systems
- Want structured state transitions

**Example**:
```
/create-state-machine robot_behavior --states idle,walking,turning,stopped --initial idle
```

**Inputs**:
- Machine name (required)
- States list (required, comma-separated)
- Initial state (required)
- Language: `cpp` or `python` (optional)
- ROS 2 integration: yes/no (optional)

**Outputs**:
- State machine class with all states
- Transition definitions
- Entry/exit actions
- Event processing
- Test file with state transition tests
- State diagram in documentation
- ROS 2 node wrapper (if requested)

---

### CI/CD

#### 5. [add-ci-workflow](./add-ci-workflow.prompt.md)

**Purpose**: Create GitHub Actions workflow for ROS 2 workspace automation.

**When to use**:
- Adding CI/CD to the project
- Need automated build/test/coverage workflows
- Want documentation generation
- Following project CI conventions

**Example**:
```
/add-ci-workflow --type test
```

**Inputs**:
- Workflow type (required): `build`, `test`, `coverage`, `docs`, or `full`
- Trigger events (optional)
- ROS distro (optional, default: `jazzy`)
- Target packages (optional, default: all)

**Outputs**:
- `.github/workflows/<workflow_name>.yml`
- Jobs for build, test, coverage, docs (based on type)
- Artifact upload configuration
- Matrix strategy (if multiple configs)
- Path filters for conditional execution

---

### Documentation

#### 6. [generate-pr-description](./generate-pr-description.prompt.md)

**Purpose**: Generate comprehensive pull request description following code-review-standards.

**When to use**:
- Creating a pull request
- Need detailed, professional PR description
- Want to follow project conventions
- Preparing for code review

**Example**:
```
/generate-pr-description
```

**Inputs**:
- Branch name (optional, uses current)
- Commit range (optional)
- Related issues (optional)
- Breaking changes flag (optional)

**Outputs**:
- Complete PR description in markdown
- Change summary by category (features, fixes, refactoring, etc.)
- Technical details and approach
- Testing strategy
- Breaking changes and migration steps
- Risk assessment
- Related issues/PRs linked
- Checklist for review

---

## How to Use Prompt Files

### In GitHub Copilot Chat (VS Code)

1. Open Copilot Chat (Ctrl+Shift+I or Cmd+Shift+I)
2. Type `/` to see available prompts
3. Select a prompt from the list or type the prompt name
4. Provide required inputs when prompted
5. Review and apply the generated output

### With Arguments

```
/<prompt-name> <required-input> --flag value
```

Example:
```
/create-ros2-package drqp_sensors --type cpp --deps rclcpp,sensor_msgs
```

### With Context

Some prompts use editor context:
- `${file}`: Current file in editor
- `${selection}`: Selected text
- `${workspaceFolder}`: Workspace root

Example:
```
# Open a source file in editor
# Then run:
/add-test-file
# It will automatically detect the current file
```

## Prompt Development

### Creating New Prompts

Follow the guidelines in [prompt.instructions.md](../ instructions/prompt.instructions.md):

1. **Frontmatter**: Include name, description, agent, tools
2. **Description**: Use WHAT/WHEN/KEYWORDS pattern for discoverability
3. **Body Structure**: Mission, Inputs, Workflow, Output, Validation, QA
4. **Clear Instructions**: Imperative mood, specific steps
5. **Quality Assurance**: Include checklist and edge cases
6. **Related Resources**: Link to skills, agents, documentation

### Testing Prompts

1. Test with representative scenarios
2. Verify all input combinations
3. Check edge case handling
4. Validate output quality
5. Ensure proper error messages

### Maintaining Prompts

- Update when dependencies or conventions change
- Review periodically for accuracy
- Coordinate with related skills and agents
- Version control changes
- Document breaking changes

## Related Resources

### Instructions

- [Prompt Guidelines](../instructions/prompt.instructions.md) - How to create prompt files
- [Engineering Principles](../instructions/engineering.instructions.md) - Code quality standards
- [Python Conventions](../instructions/python.instructions.md) - Python-specific guidelines

### Skills

- [ros2-workspace-build](../skills/ros2-workspace-build/SKILL.md) - Build ROS 2 packages
- [ros2-workspace-testing](../skills/ros2-workspace-testing/SKILL.md) - Test and coverage
- [ros2-dependency-management](../skills/ros2-dependency-management/SKILL.md) - Dependencies
- [ros2-environment-setup](../skills/ros2-environment-setup/SKILL.md) - Environment
- [code-review-standards](../skills/code-review-standards/SKILL.md) - PR and review standards
- [find-test-files](../skills/find-test-files/SKILL.md) - Locate test files

### Agents

- [Task Planner](../agents/task-planner.agent.md) - Create implementation plans
- [TDD Red](../agents/tdd-red.agent.md) - Write failing tests
- [TDD Green](../agents/tdd-green.agent.md) - Implement to pass tests
- [TDD Refactor](../agents/tdd-refactor.agent.md) - Improve code quality
- [Principal Engineer](../agents/principal-engineer.agent.md) - Code review and architecture

### Documentation

- [AGENTS.md](../../AGENTS.md) - All available agents and skills
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) - Official ROS 2 docs
- [GitHub Actions](https://docs.github.com/en/actions) - CI/CD documentation

## Troubleshooting

### Prompt Not Appearing

- Check that file has `.prompt.md` extension
- Verify frontmatter YAML is valid
- Reload VS Code window
- Check Copilot Chat is enabled

### Prompt Not Activating Automatically

- Improve `description` field with specific keywords
- Include trigger phrases in description
- Mention specific file types or scenarios

### Unexpected Behavior

- Review prompt instructions for clarity
- Check tool permissions are correct
- Verify input validation is working
- Test with edge cases

### Output Quality Issues

- Refine workflow steps
- Add more examples
- Improve validation criteria
- Update quality assurance checklist

## Contributing

When adding new prompts:

1. Follow the [prompt.instructions.md](../instructions/prompt.instructions.md) specification
2. Test thoroughly with real scenarios
3. Update this README with the new prompt
4. Link to related skills and agents
5. Provide clear examples

For questions or improvements, create a GitHub issue or pull request.
