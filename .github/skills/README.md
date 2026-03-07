# Agent Skills

This directory contains Agent Skills for GitHub Copilot - self-contained workflows and resources that teach AI agents specialized capabilities.

## What Are Agent Skills?

Agent Skills are automatically discovered and loaded by GitHub Copilot based on the relevance of your request. Each skill includes:

- **Frontmatter**: Metadata with name, description, and license
- **Body**: Step-by-step workflows, prerequisites, and troubleshooting
- **Resources** (optional): Scripts, templates, examples, and documentation

See [Agent Skills Guidelines](../instructions/agent-skills.instructions.md) for complete documentation.

## Skill Validation

All skills in this directory are automatically validated to ensure they follow best practices and can be effectively discovered by Copilot.

### Validation Rules

Skills are validated against the following rules:

#### 1. Frontmatter Requirements

- **name**: Required, lowercase with hyphens, max 64 characters (e.g., `my-skill-name`)
- **description**: Required, 10-1024 characters, must include WHAT/WHEN/KEYWORDS pattern
- **license**: Optional but recommended (e.g., `Complete terms in LICENSE.txt` or SPDX identifier)

#### 2. Description Pattern (WHAT/WHEN/KEYWORDS)

The description is the PRIMARY mechanism for skill discovery. It must include:

- **WHAT**: Clear statement of capabilities
- **WHEN**: Specific triggers and use cases (e.g., "Use when asked to...", "for debugging...")
- **KEYWORDS**: Minimum 5 unique technical terms, actions, or domain concepts

**Errors:**

- ❌ Vague terms: helpers, utilities, tools, stuff, things, misc, various, general
- ❌ Missing WHEN guidance (no trigger phrases)
- ❌ Too few keywords (< 5 unique terms)

**Good Example:**

```yaml
description: Toolkit for testing local web applications using Playwright browser automation. Use when asked to verify frontend functionality, debug UI behavior, capture browser screenshots, check for visual regressions, or view browser console logs. Supports Chrome, Firefox, and WebKit browsers.
```

**Bad Example:**

```yaml
description: Web testing helpers
```

#### 3. Content Structure Requirements

Required sections in the body:

- **When to Use This Skill**: List of specific scenarios
- **Prerequisites**: Required tools, dependencies, environment setup

Recommended sections:

- **Troubleshooting**: Common issues and solutions (preferably as a table)
- **Step-by-Step Workflows**: At least 2 workflow sections with numbered steps

#### 4. Uniqueness Requirements

- No duplicate skill names across all skills
- Avoid excessive keyword overlap (>70% similarity) that might cause activation conflicts

#### 5. Cross-Reference Requirements

- Skills should be listed in this README
- Internal links (to scripts, references, templates) must point to valid files

### Running Validation

**Validate all skills:**

```bash
validate_skills
```

**Validate specific skill:**

```bash
validate_skills .github/skills/my-skill/
```

**Show warnings and recommendations:**

```bash
validate_skills --recommend
```

**CI mode (exit with error on failures):**

```bash
validate_skills --ci
```

### CI/CD Integration

Skills are validated automatically in GitHub Actions when:

- Opening or updating pull requests (`pull_request`)
- Pushing to the `main` branch (`push`)
- A merge queue requests checks (`merge_group` with `checks_requested`)

See [`.github/workflows/copilot.yml`](../workflows/copilot.yml) for details.

## Example Skills

Example "good" and "bad" skills are defined inline in the unit tests using
temporary directories (via `tmp_path` fixtures), rather than checked-in
fixture files. Refer to the tests under `py_packages/validate_agents/tests/validate_skills/` for concrete examples
of valid and invalid `SKILL.md` definitions.

## Skill Catalog

### ROS 2 Workflow Skills

- **ros2-environment-setup** - Environment initialization, devcontainer, venv. `.github/skills/ros2-environment-setup/`
- **ros2-workspace-build** - Build ROS 2 packages with colcon. `.github/skills/ros2-workspace-build/`
- **ros2-workspace-testing** - Run tests, coverage. `.github/skills/ros2-workspace-testing/`
- **ros2-dependency-management** - rosdep, pip, package.xml. `.github/skills/ros2-dependency-management/`
- **ros2-launch-management** - Launch files, parameters. `.github/skills/ros2-launch-management/`
- **ros2-lifecycle-management** - Lifecycle node states. `.github/skills/ros2-lifecycle-management/`
- **ros2-parameter-tuning** - Parameter configuration. `.github/skills/ros2-parameter-tuning/`
- **ros2-diagnostics** - Debug topics, services, nodes. `.github/skills/ros2-diagnostics/`

### Package and Code Creation Skills

- **create-ros2-package** - New ROS 2 package (C++, Python, mixed). `.github/skills/create-ros2-package/`
- **add-test-file** - Unit or integration test scaffolding. `.github/skills/add-test-file/`
- **implement-publisher-subscriber** - Pub/sub nodes, QoS. `.github/skills/implement-publisher-subscriber/`
- **create-state-machine** - FSM implementation. `.github/skills/create-state-machine/`
- **add-ci-workflow** - GitHub Actions for ROS 2. `.github/skills/add-ci-workflow/`

### Review and PR Skills

- **code-review-standards** - PR descriptions, review practices. `.github/skills/code-review-standards/`
- **generate-pr-description** - Generate PR body. `.github/skills/generate-pr-description/`
- **pr-feedback-resolution** - Address PR feedback, CI, CodeQL, coverage. `.github/skills/pr-feedback-resolution/`
- **open-pr** - Create GitHub PR. `.github/skills/open-pr/`
- **update-branch** - Sync branch with main. `.github/skills/update-branch/`
- **find-test-files** - Locate test files. `.github/skills/find-test-files/`

## Creating New Skills

1. Create a new directory: `.github/skills/my-new-skill/`
2. Create `SKILL.md` with proper frontmatter and structure
3. Add any resources (scripts, templates, references) in subdirectories
4. Run validation: `validate_skills .github/skills/my-new-skill/`
5. Fix any errors or warnings
6. Add skill to this README's catalog section
7. Commit and push (pre-commit hook will validate automatically)

## Resources

- [Agent Skills Specification](https://agentskills.io/)
- [VS Code Agent Skills Documentation](https://code.visualstudio.com/docs/copilot/customization/agent-skills)
- [Agent Skills Guidelines](../instructions/agent-skills.instructions.md)
- [Validation package](../../py_packages/validate_agents/)
