# Agents Guidelines

NEVER use "$TMPDIR" env variable.
ALWAYS use "./.tmp" (relative to the repo root) for temporary files; create it if it does not exist.
NEVER use GitHub API or GitHub MCP tools to update branch refs or push branch contents. Use local git branch workflows instead; if push authentication is unavailable, stop and report the blocker rather than updating the branch remotely via API.

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
11. **When available in VS Code, use `vscode/askQuestions`** for all yes/no and multiple-choice user questions
12. **For `unittest` ROS tests, never call `rclpy.init()` in `setUpClass`**. Initialize ROS in `setUp()` and register `self.addCleanup(rclpy.shutdown)` immediately so shutdown still runs if setup fails. Prefer `self.addCleanup(...)` for node, client, subscription, and action cleanup as well.

### When in Doubt

Consult the **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** agent for architecture, design decisions, and implementation strategies.

## Catalog Locations

- **Claude**: `.claude/agents/`, `.claude/commands/`, `.claude/instructions/` (symlinks to `.github/`)
- **Cursor**: `.cursor/agents/`, `.cursor/skills/`, `.cursor/rules/` (symlinks to `.github/`)
- **Codex**: `.codex/agents/`, `.codex/skills/`, `.codex/instructions/` (symlinks to `.github/`)

Update `.github` sources; symlinks pick up changes automatically.

## Available Agents

### Development

- **[Principal Engineer](/.github/agents/principal-engineer.agent.md)** - Senior guidance, architecture, pragmatic implementation

### Testing & Quality

- **[TDD Red](/.github/agents/tdd-red.agent.md)** - Write failing tests before implementation
- **[TDD Green](/.github/agents/tdd-green.agent.md)** - Implement minimal code to satisfy requirements
- **[TDD Refactor](/.github/agents/tdd-refactor.agent.md)** - Improve code quality while maintaining tests

## Remote Agent Guidelines

When addressing PR review comments, CI failures, CodeQL findings, or coverage gaps, use the [pr-feedback-resolution](/.github/skills/pr-feedback-resolution/) skill.

## Available Skills

Browse [`.github/skills/`](/.github/skills/). See [README](/.github/skills/README.md) for validation.

Key skills: ros2-environment-setup, ros2-workspace-build, ros2-workspace-testing, ros2-dependency-management, code-review-standards, pr-feedback-resolution, create-ros2-package, add-test-file, implement-publisher-subscriber, create-state-machine, generate-pr-description.

## Code Review Standards

[code-review-standards](/.github/skills/code-review-standards/)

## Cursor Cloud Sessions

**Load the microVM-sandbox instructions** — Read and follow [microVM-sandbox](/.github/instructions/microVM-sandbox.instructions.md) before running any build, test, or lint command.
