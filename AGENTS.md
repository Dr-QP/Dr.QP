# Agents Guidelines

NEVER use "$TMPDIR" env variable.
ALWAYS use "./.tmp" (relative to the repo root) for temporary files; create it if it does not exist.
NEVER use GitHub API or GitHub MCP tools to update branch refs or push branch contents. Use local git branch workflows instead; if push authentication is unavailable, stop and report the blocker rather than updating the branch remotely via API.

## Best Practices for Agents

0. NEVER change git config on local or global level unless explicitly instructed. NEVER switch/change remote.
1. **Always use scripts/with-ros-env.sh** to run ROS commands like `colcon build` or `colcon test`
2. **Use incremental builds** (`--packages-up-to <pkg>`) during development
3. **Test specific packages** (`--packages-select <pkg>`) for rapid iteration
4. **Escalate to a containerized environment when local ROS 2 is missing — never give up after a local failure.** If `colcon`/`ros2` is not found, or `scripts/with-ros-env.sh` fails because `/opt/ros/$ROS_DISTRO/setup.bash` does not exist, the host has no ROS 2 and retrying locally cannot succeed. Escalate in this order: (a) Docker daemon available (e.g. Cursor cloud sessions) → use the [microvm-sandbox](/.claude/skills/microvm-sandbox/SKILL.md) skill to run the command through `devcontainer exec`; (b) no Docker daemon (e.g. Codex Tasks) → use the [remote-codespace-session](/.claude/skills/remote-codespace-session/SKILL.md) skill to run it on a GitHub Codespace over SSH. Only report a blocker if both escalation paths are unavailable (e.g. no `gh` auth)
5. **Only run full builds/tests** when explicitly requested
6. **Collect test output** from `log/latest_test/<package_name>/stdout_stderr.log` or `streams.log` (timestamped) in same folder.
7. **Check build logs** in `log/latest_build/` if builds fail
8. **Use `--symlink-install`** for Python coverage and hot-reload
9. **Enable coverage** with `--mixin coverage-pytest` when testing
10. **Re-run failed tests** with `--packages-select-test-failures`
11. **When available in VS Code, use `vscode/askQuestions`** for all yes/no and multiple-choice user questions
12. **Do not re-declare, bind, or explicitly forward launch arguments owned by included ROS launch files**. Externally set launch arguments are available to included launch descriptions through the launch context; only use `DeclareLaunchArgument`, `LaunchConfiguration`, or `launch_arguments={...}` entries for arguments owned by the current launch file.
13. **Codex: run ROS build/test/lint commands with sandbox escalation** (`require_escalated`) because the managed sandbox blocks dependency downloads, ROS domain sockets, Gazebo/launch process behavior, and ROS log writes under `/root/.ros`. Still use `scripts/with-ros-env.sh`, incremental build/test selectors, and explain the escalation reason in the request.
14. **`@pytest.mark.flaky` on a `@pytest.mark.launch(...)` (`launch_pytest`) test is safe out of the box.** Stock `launch_pytest` re-wraps `pyfuncitem.obj` in place on every call, so a naive retry would re-wrap the previous attempt's stale wrapper and crash with a closed/duplicated `asyncio` event loop instead of retrying — this workspace's vendored `launch_pytest` (`packages/vendor/launch/launch_pytest`, see `source-info.yaml`) fixes that at the source, so no per-package shim or `test/conftest.py` registration is needed. See `drqp_launch_testing/test/shutdown_behavior/SPEC.md` (combo 6) for the root cause and proof. Retry only usefully rescues **function-scoped** launch fixtures (combo 5): `pytest-retry` does not re-run a `module`-scoped fixture's setup on retry, so a combo 4 (module-scoped shared simulation) `shutdown=True` test is crash-safe but not relaunch-safe — don't add `flaky` there expecting it to help.

### When in Doubt

Consult the **[Principal Engineer](/.claude/agents/principal-engineer.agent.md)** agent for architecture, design decisions, and implementation strategies.

## Coding Conventions

### Python

- Follow **PEP 8**: 4 spaces per indentation level, lines ≤ 79 characters, descriptive names
- Use type hints (PEP 484, `typing` module) and PEP 257 docstrings placed immediately after `def`/`class`
- Format and autofix with **ruff first** (`scripts/python-reformat.sh`), then verify compliance with **`ament_flake8`** — the CI gate — via `scripts/python-lint-check.sh`. Never judge style with pure `flake8`: its stock defaults (79-char limit, stock isort grouping) produce false positives that do not match this repo (`ruff.toml` uses a 99-char limit) and do not fail CI. Full workflow in the [python-format-lint](/.claude/skills/python-format-lint/) skill
- Add ROS and workspace packages to Ruff's `lint.isort.known-third-party` configuration when that classification aligns its import order with `ament_flake8`. Use a targeted `# noqa: <rule>` only for a formatter-required incompatibility such as `E203` on a slice; do not disable rules for a whole file or package
- **Exception handling**: never write empty handlers (`except ...: pass`). Handle expected exceptions explicitly by at least one of: logging context, returning a safe fallback value, re-raising with context, or raising `SystemExit` for CLI interruption paths (`raise SystemExit(130)` for user interrupts). If an exception must be intentionally ignored, document the reason in a comment and keep the ignored scope minimal. Prefer specific exception types over broad `except Exception`

### Python Testing

- **Always use `pytest`** — never `unittest`
- For ROS 2 launch or node integration tests use `launch_pytest`: decorate `generate_test_description` with `@launch_pytest.fixture` and tests with `@pytest.mark.launch(fixture=generate_test_description)` — full rules in the [launch-testing](/.claude/skills/launch-testing/) skill
- Prefer multiple smaller, focused test files over large monolithic ones

### Local Scripts, Docs, and Notebooks (`scripts/`, `docs/`)

- Use the `.venv` virtual environment for executing scripts locally; define workspace dependencies in `pyproject.toml` and sync `.venv` with `uv`. Avoid global package installations
- Ensure code snippets in documentation are executable and tested

### C++

- Follow the C++ Core Guidelines with modern C++ (C++17 or later): RAII for resource management, value semantics by default, smart pointers instead of raw pointers, standard library containers and algorithms
- Make ownership explicit in API design; focus on correctness first, then optimize with evidence

### ROS 2

- Follow ROS 2 naming conventions for packages, nodes, topics, services, and actions
- Document package dependencies in `package.xml`, and parameter defaults and constraints
- Include launch files for complex multi-node systems and integration tests for node interactions

## Catalog Locations

- **Claude** (canonical source of truth): `.claude/agents/`, `.claude/skills/`
- **Codex**: `.codex/agents/` (trampolines to `.claude/agents/`), `.codex/skills/` (symlink to `.claude/skills/`)
- **Cursor**: `.cursor/agents/`, `.cursor/skills/` (symlinks to `.claude/`)

Update `.claude` sources; symlinks pick up changes automatically. When adding or renaming an agent, also update its `.codex/agents/` trampoline.

\*\*Always edit the `AGENTS.md`, not just `CLAUDE.md` to cover all agents.

## Spikes

When doing investigation work aka spikes document your findings in specs under `docs/agents/specs/<spike-topic>` in a structured way, create new subfolder `<spike-topic>` for the subject of investigation. Create `Readme.md` with raw findings, including list of issues with assigned priorities. Create a series of spec files with implementation guidance.
