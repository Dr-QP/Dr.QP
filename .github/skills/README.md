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
python3 scripts/validate-skills.py
```

**Validate specific skill:**
```bash
python3 scripts/validate-skills.py .github/skills/my-skill/SKILL.md
```

**Show warnings and recommendations:**
```bash
python3 scripts/validate-skills.py --recommend
```

**CI mode (exit with error on failures):**
```bash
python3 scripts/validate-skills.py --ci
```

### VS Code Tasks

Run validation from VS Code:
1. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
2. Type "Tasks: Run Task"
3. Select "Validate Skills" or "Validate Skills (CI Mode)"

### Pre-commit Hook

Skills are automatically validated before commit. To run manually:
```bash
pre-commit run validate-skills
```

### CI/CD Integration

Skills are validated automatically in GitHub Actions when:
- Pushing to main or develop branches
- Opening/updating pull requests
- Manually triggered via workflow_dispatch

See [`.github/workflows/validate-skills.yml`](../workflows/validate-skills.yml) for details.

## Example Skills

Example "good" and "bad" skills are defined inline in the unit tests using
temporary directories (via `tmp_path` fixtures), rather than checked-in
fixture files. Refer to the tests under `scripts/validate_skills/tests/` for concrete examples
of valid and invalid `SKILL.md` definitions.

## Skill Catalog

Currently, this repository has no production skills. Skills will be listed here as they are added.

<!--
Example entry format:

### skill-name

**Description**: Brief description of what the skill does.

**Use when**: Specific trigger scenarios.

**Location**: `.github/skills/skill-name/`
-->

## Creating New Skills

1. Create a new directory: `.github/skills/my-new-skill/`
2. Create `SKILL.md` with proper frontmatter and structure
3. Add any resources (scripts, templates, references) in subdirectories
4. Run validation: `python3 scripts/validate-skills.py .github/skills/my-new-skill/`
5. Fix any errors or warnings
6. Add skill to this README's catalog section
7. Commit and push (pre-commit hook will validate automatically)

## Resources

- [Agent Skills Specification](https://agentskills.io/)
- [VS Code Agent Skills Documentation](https://code.visualstudio.com/docs/copilot/customization/agent-skills)
- [Agent Skills Guidelines](../instructions/agent-skills.instructions.md)
- [Validation Script Source](../../scripts/validate-skills.py)
