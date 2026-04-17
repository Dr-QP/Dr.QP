---
description: 'Guidelines for creating high-quality Agent Skills for GitHub Copilot'
applyTo: '**/.github/skills/**/SKILL.md,**/.cursor/skills/**/SKILL.md,**/.claude/skills/**/SKILL.md,**/.agents/skills/**/SKILL.md'
---

# Agent Skills File Guidelines

Instructions for creating effective and portable Agent Skills that enhance GitHub Copilot with specialized capabilities, workflows, and bundled resources.

## What Are Agent Skills?

Agent skills are folders of instructions, scripts, and resources that Copilot can load when relevant to improve performance on specialized tasks. Unlike custom instructions, which are best for simple repository-wide guidance, skills are meant for task-specific workflows that should only be loaded when needed.

Key characteristics:
- **Portable**: Works across VS Code, Copilot CLI, and Copilot coding agent
- **On-demand**: Copilot decides to use a skill based on the prompt and the skill's `description`
- **Resource-bundled**: A skill directory can include scripts, examples, and supplementary Markdown resources
- **Scoped**: Use skills for detailed task workflows, not general coding standards

## Directory Structure

GitHub's documented skill locations are:

| Location | Scope | Recommendation |
|----------|-------|----------------|
| `.github/skills/<skill-name>/` | Project/repository | Recommended for project skills |
| `.claude/skills/<skill-name>/` | Project/repository | Legacy, for backward compatibility |
| `.agents/skills/<skill-name>/` | Project/repository | Supported alternative project location |
| `~/.copilot/skills/<skill-name>/` | Personal (user-wide) | Recommended for personal skills |
| `~/.claude/skills/<skill-name>/` | Personal (user-wide) | Legacy, for backward compatibility |
| `~/.agents/skills/<skill-name>/` | Personal (user-wide) | Supported alternative personal location |

Each skill **must** have its own subdirectory containing at minimum a `SKILL.md` file.

Repository note:
This repository may also mirror skills into `.cursor/skills/` for editor compatibility, but that is a repository convention rather than a GitHub Docs skill location.

## Required SKILL.md Format

### Frontmatter (Required)

```yaml
---
name: github-actions-failure-debugging
description: Guide for debugging failing GitHub Actions workflows. Use this when asked to debug failing GitHub Actions workflows.
license: Complete terms in LICENSE.txt
---
```

| Field | Required | Constraints |
|-------|----------|-------------|
| `name` | Yes | Unique identifier, lowercase, hyphens for spaces, usually matches the skill directory name |
| `description` | Yes | Describe what the skill does and when Copilot should use it |
| `license` | No | Describe the license that applies to the skill |
| `allowed-tools` | No | Optional tool pre-approval list for skills that need to run tools such as `shell` |

Important points:
- The file name must be exactly `SKILL.md`
- The skill directory name should be lowercase and use hyphens for spaces
- The Markdown body should contain instructions, examples, and guidelines Copilot can follow

### Description Best Practices

**CRITICAL**: Copilot decides when to use a skill based on the user's prompt and the skill's `description`. If the description is vague, the skill is unlikely to be selected at the right time.

**What to include in description** — Follow the **WHAT/WHEN/KEYWORDS pattern**:
1. **WHAT** the skill does (capabilities)
2. **WHEN** to use it (specific triggers, scenarios, file types, or user requests)
3. **Keywords** that users might mention in their prompts

**Example of excellent description:**
```yaml
description: Guide for debugging failing GitHub Actions workflows. Use this when asked to debug failing GitHub Actions workflows.
```

**Example of poor description:**
```yaml
description: Web testing helpers
```
This fails because it does not clearly say what the skill does or when Copilot should use it.

### Common Skill Discovery Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Skill never activates | Vague or generic description | Rewrite the description so it states the task and trigger clearly |
| Activates on wrong requests | Description is too broad | Narrow the trigger phrases and task scope |
| Conflicting skill activations | Multiple skills describe the same use case | Differentiate the descriptions and intended scenarios |
| Skill runs without enough context | Body lacks actionable instructions | Add explicit steps, examples, and usage notes |

### Body Content

The body contains the instructions Copilot receives after the skill is selected. GitHub Docs do not require a fixed section layout, but the body should be concrete and operational.

| Section | Purpose |
|---------|---------|
| `# Title` | Brief overview of what this skill enables |
| `## When to Use This Skill` | List of scenarios (reinforces description triggers) |
| `## Prerequisites` | Required tools, dependencies, environment setup |
| `## Step-by-Step Workflows` | Numbered steps for common tasks |
| `## Troubleshooting` | Common issues and solutions table |
| `## References` | Links to bundled docs or external resources |

## Bundling Resources

When a skill is invoked, Copilot automatically discovers the files in that skill's directory and makes them available alongside the instructions. That means you can add scripts, supplementary Markdown files, examples, or templates and reference them directly from `SKILL.md`.

GitHub Docs do not require a fixed folder taxonomy. These subfolders are still useful conventions when they improve organization:

| Folder | Purpose | Loaded into Context? | Example Files |
|--------|---------|---------------------|---------------|
| `scripts/` | Executable automation that performs specific operations | When executed | `helper.py`, `validate.sh`, `build.ts` |
| `references/` | Documentation the AI agent reads to inform decisions | Yes, when referenced | `api_reference.md`, `schema.md`, `workflow_guide.md` |
| `templates/` | Starter code or scaffolds that the AI agent modifies | Yes, when referenced | `viewer.html`, `hello-world/`, `config.template` |
| `assets/` | Static supporting files used as-is | Usually no | `logo.png`, `baseline.png`, `report-template.html` |

### Directory Structure Example

```
.github/skills/my-skill/
├── SKILL.md
├── LICENSE.txt
├── scripts/
│   └── helper.py
├── references/
│   └── workflow-guide.md
└── templates/
    └── scaffold.py
```

> **LICENSE.txt**: When creating a skill, download the Apache 2.0 license text from https://www.apache.org/licenses/LICENSE-2.0.txt and save as `LICENSE.txt`. Update the copyright year and owner in the appendix section.

### Referencing Resources in SKILL.md

Use relative paths when referring to skill resources from `SKILL.md`.

```markdown
## Available Scripts

Run the [helper script](./scripts/helper.py) to automate the workflow.

See the [workflow guide](./references/workflow-guide.md) for the full procedure.

Use the [scaffold](./templates/scaffold.py) as a starting point.
```

## Running Scripts Safely

If a skill needs to run a script, include the script in the skill directory and describe exactly when and how Copilot should run it.

Example layout:

```text
.github/skills/image-convert/
├── SKILL.md
└── convert-svg-to-png.sh
```

Optional frontmatter for pre-approving tools:

```yaml
---
name: image-convert
description: Converts SVG images to PNG format. Use when asked to convert SVG files.
allowed-tools: shell
---
```

Security guidance:
- Only pre-approve `shell` or `bash` if you fully trust the skill and every referenced script
- Pre-approving terminal tools removes the normal confirmation step for command execution
- When in doubt, omit `shell` and `bash` from `allowed-tools` so Copilot must ask first

Instruction pattern for scripts:

```markdown
When asked to convert an SVG to PNG, run the `convert-svg-to-png.sh` script
from this skill's base directory, passing the input SVG file path as the
first argument.
```

## How Copilot Loads Skills

Copilot uses skills as follows:

| Level | What Loads | When |
|-------|------------|------|
| 1. Discovery | Skill metadata, especially `description` | Used to decide whether the skill is relevant |
| 2. Instructions | `SKILL.md` body | Loaded when Copilot chooses the skill |
| 3. Resources | Scripts and supporting files in the skill directory | Available alongside the instructions when needed |

Practical implication:
- Put concise routing logic in the `description`
- Put operational guidance in the body
- Keep supporting files close to the skill and reference them explicitly

## Content Guidelines

### Writing Style

- Use imperative mood: "Run", "Create", "Configure" (not "You should run")
- Be specific and actionable
- Include exact commands with parameters
- Show expected outputs where helpful
- Keep sections focused and scannable

### Script Requirements

When including scripts, prefer cross-platform languages:

| Language | Use Case |
|----------|----------|
| Python | Complex automation, data processing |
| pwsh | PowerShell Core scripting |
| Node.js | JavaScript-based tooling |
| Bash/Shell | Simple automation tasks |

Best practices:
- Include help/usage documentation (`--help` flag)
- Handle errors gracefully with clear messages
- Avoid storing credentials or secrets
- Use relative paths where possible
- Keep script invocation instructions explicit in `SKILL.md`

### When to Bundle Scripts

Include scripts in your skill when:
- The same code would be rewritten repeatedly by the agent
- Deterministic reliability is critical (e.g., file manipulation, API calls)
- Complex logic benefits from being pre-tested rather than generated each time
- The operation has a self-contained purpose that can evolve independently
- Testability matters — scripts can be unit tested and validated
- Predictable behavior is preferred over dynamic generation

Scripts enable evolution: even simple operations benefit from being implemented as scripts when they may grow in complexity, need consistent behavior across invocations, or require future extensibility.

### Security Considerations

- Scripts rely on existing credential helpers (no credential storage)
- Do not pre-approve `shell` or `bash` in `allowed-tools` unless the skill is trusted and reviewed
- Include `--force` flags only for destructive operations
- Warn users before irreversible actions
- Document any network operations or external calls

## Skills Versus Custom Instructions

Use custom instructions for simple guidance that applies to almost every task, such as repository coding standards or broad workflow expectations.

Use skills for detailed, reusable task playbooks that should only enter context when they are relevant.

## Adding Existing Skills

You can add a skill created by someone else by downloading a directory that contains `SKILL.md` and any related files, then moving that directory into one of the supported skill locations.

Useful source:
- https://awesome-copilot.github.com/skills/

## Common Patterns

### Parameter Table Pattern

Document parameters clearly:

```markdown
| Parameter | Required | Default | Description |
|-----------|----------|---------|-------------|
| `--input` | Yes | - | Input file or URL to process |
| `--action` | Yes | - | Action to perform |
| `--verbose` | No | `false` | Enable verbose output |
```

## Validation Checklist

Before publishing a skill:

- [ ] Skill directory is in a supported location such as `.github/skills/`, `.claude/skills/`, `.agents/skills/`, or `~/.copilot/skills/`
- [ ] `SKILL.md` has valid frontmatter with `name` and `description`
- [ ] File is named exactly `SKILL.md`
- [ ] Skill directory name is lowercase with hyphens
- [ ] `description` clearly states **WHAT** it does, **WHEN** to use it, and relevant **KEYWORDS**
- [ ] Body includes when to use, prerequisites, and step-by-step workflows
- [ ] Any `allowed-tools` entries were reviewed deliberately, especially `shell` or `bash`
- [ ] Scripts include help documentation and error handling
- [ ] Relative paths used for all resource references
- [ ] No hardcoded credentials or secrets

## Workflow Execution Pattern

When executing multi-step workflows, create a TODO list where each step references the relevant documentation:

```markdown
## TODO
- [ ] Step 1: Configure environment - see [workflow-setup.md](./references/workflow-setup.md#environment)
- [ ] Step 2: Build project - see [workflow-setup.md](./references/workflow-setup.md#build)
- [ ] Step 3: Deploy to staging - see [workflow-deployment.md](./references/workflow-deployment.md#staging)
- [ ] Step 4: Run validation - see [workflow-deployment.md](./references/workflow-deployment.md#validation)
- [ ] Step 5: Deploy to production - see [workflow-deployment.md](./references/workflow-deployment.md#production)
```

This ensures traceability and allows resuming workflows if interrupted.

## Related Resources

- [GitHub Docs: Adding agent skills for GitHub Copilot](https://docs.github.com/en/copilot/how-tos/use-copilot-agents/cloud-agent/add-skills)
- [GitHub Docs: About agent skills](https://docs.github.com/en/copilot/concepts/agents/about-agent-skills)
- [VS Code Agent Skills Documentation](https://code.visualstudio.com/docs/copilot/customization/agent-skills)
- [Awesome Copilot Skills](https://github.com/github/awesome-copilot/blob/main/docs/README.skills.md)
