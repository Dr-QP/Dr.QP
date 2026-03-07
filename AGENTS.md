# Agents Guidelines

NEVER use "$TMPDIR" env variable.
ALWAYS use "./.tmp" (relative to the repo root) for temporary files; create it if it does not exist.

## Best Practices for Agents

1. **Always source scripts/setup.bash** before build or test
2. **Use incremental builds** (`--packages-up-to <pkg>`) during development
3. **Test specific packages** (`--packages-select <pkg>`) for rapid iteration
4. **Use devcontainer** when running as remote agent (see [Cursor Cloud Sessions](#cursor-cloud-sessions) below)
5. **Only run full builds/tests** when explicitly requested
6. **Collect test output** from `build/<package_name>/test_results/`
7. **Check build logs** in `log/latest_build/` if builds fail
8. **Use `--symlink-install`** for Python coverage and hot-reload
9. **Enable coverage** with `--mixin coverage-pytest` when testing
10. **Re-run failed tests** with `--packages-select-test-failures`

### When in Doubt

Consult the **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** agent for architecture, design decisions, and implementation strategies.

## Catalog Locations

- **Cursor**: `.cursor/agents/`, `.cursor/skills/`, `.cursor/rules/` (symlinks to `.github/`)
- **Codex**: `.codex/agents/`, `.codex/skills/`, `.codex/instructions/`, (symlinks to `.github/`)

Update `.github` sources; symlinks pick up changes automatically.

## Available Agents

### Planning & Analysis

- **[Task Planner](/.github/agents/task-planner.agent.md)** - Create actionable implementation plans
- **[Task Researcher](/.github/agents/task-researcher.agent.md)** - Comprehensive project analysis and research
- **[Technical Spike Researcher](/.github/agents/technical-spike-researcher.agent.md)** - Research and validate technical spike documents
- **[Issue Refiner](/.github/agents/issue-refiner.agent.md)** - Refine requirements with acceptance criteria and edge cases

### Development

- **[Code Alchemist](/.github/agents/code-alchemist.agent.md)** - Transform code with Clean Code and SOLID
- **[C++ Expert](/.github/agents/cpp-expert.agent.md)** - Expert C++ guidance, modern practices
- **[React Expert](/.github/agents/react-expert.agent.md)** - React frontend with hooks and performance
- **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** - Senior guidance, architecture, pragmatic implementation

### Testing & Quality

- **[TDD Red](/.github/agents/tdd-red.agent.md)** - Write failing tests before implementation
- **[TDD Green](/.github/agents/tdd-green.agent.md)** - Implement minimal code to satisfy requirements
- **[TDD Refactor](/.github/agents/tdd-refactor.agent.md)** - Improve code quality while maintaining tests

### Debugging & Review

- **[Debugger](/.github/agents/debug.agent.md)** - Debug and fix bugs
- **[Security Sentinel](/.github/agents/security-sentinel.agent.md)** - Review for security issues
- **[Tech Debt Remediator](/.github/agents/tech-debt-remediator.agent.md)** - Technical debt remediation plans

### Support

- **[Prompt Builder](/.github/agents/prompt-builder.agent.md)** - Prompt engineering and validation
- **[Mentor](/.github/agents/mentor.agent.md)** - Socratic questioning and critical thinking
- **[Technical Content Evaluator](/.github/agents/technical-content-evaluator.agent.md)** - Review technical materials
- **[Custom Agent Foundry](/.github/agents/custom-agent-foundry.agent.md)** - Design VS Code custom agents
- **[Deep Thinker](/.github/agents/deep-thinker.agent.md)** - Advanced reasoning for complex problems

## Remote Agent Guidelines

When addressing PR review comments, CI failures, CodeQL findings, or coverage gaps, use the [pr-feedback-resolution](/.github/skills/pr-feedback-resolution/) skill.

## Available Skills

Browse [`.github/skills/`](/.github/skills/). See [README](/.github/skills/README.md) for validation.

Key skills: ros2-environment-setup, ros2-workspace-build, ros2-workspace-testing, ros2-dependency-management, code-review-standards, pr-feedback-resolution, create-ros2-package, add-test-file, implement-publisher-subscriber, create-state-machine, generate-pr-description.

## Code Review Standards

[code-review-standards](/.github/skills/code-review-standards/)

## Cursor Cloud Sessions

**Load the microVM-sandbox instructions** — Read and follow [microVM-sandbox](/.github/instructions/microVM-sandbox.instructions.md) before running any build, test, or lint command.
