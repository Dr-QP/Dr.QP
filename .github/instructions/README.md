# Instructions Reference Guide

Quick navigation for agents, skills, and prompts guidance in this workspace.

## Quick Find

**Start here for:**

| Need | File | Purpose |
|------|------|---------|
| General code quality & design | [engineering.instructions.md](engineering.instructions.md) | SOLID, Clean Code, testing, language conventions |
| Python-specific guidance | [python.instructions.md](python.instructions.md) | PEP 8, type hints, docstrings |
| Creating custom agents | [agents.instructions.md](agents.instructions.md) | Agent frontmatter, tools, prompts, handoffs |
| Creating agent skills | [agent-skills.instructions.md](agent-skills.instructions.md) | Skill structure, discovery, bundling resources |
| Creating prompt files | [prompt.instructions.md](prompt.instructions.md) | Prompt syntax, variables, output expectations |

## Instruction Files Overview

### [engineering.instructions.md](engineering.instructions.md)
**Scope**: All code, agents, skills, prompts  
**Topics**:
- Core engineering principles (SOLID, Clean Code, DRY, KISS, YAGNI)
- Implementation best practices and error handling
- Testing strategies and pyramid
- Language-specific conventions (C++, Python, ROS 2)
- Technical debt management
- Code review standards
- Quality assurance checklist

**Use when**: You need general guidance on code quality, design, testing, or principles

### [python.instructions.md](python.instructions.md)
**Scope**: Python code (`.py` files)  
**Topics**:
- PEP 8 style guide compliance
- Type hints and annotations (PEP 484)
- Docstring conventions (PEP 257)
- Python-specific patterns and examples

**Use when**: Writing Python code; references engineering.instructions.md for general principles

### [agents.instructions.md](agents.instructions.md)
**Scope**: Custom agent files (`.agent.md`)  
**Topics**:
- Agent file structure and naming
- Frontmatter fields (description, name, tools, model, target, infer)
- Tool configuration and MCP servers
- Handoffs for guided workflows
- Sub-agent orchestration patterns
- Agent prompt structure and variables

**Use when**: Creating or updating custom agents; 36KB comprehensive reference

### [agent-skills.instructions.md](agent-skills.instructions.md)
**Scope**: Agent skills (`.github/skills/<name>/SKILL.md`)  
**Topics**:
- Skill directory structure
- Frontmatter format (name, description, license)
- Description best practices (WHAT/WHEN/KEYWORDS pattern)
- Bundling resources (scripts, references, assets, templates)
- Progressive loading architecture
- Validation checklist

**Use when**: Creating or updating agent skills

### [prompt.instructions.md](prompt.instructions.md)
**Scope**: Prompt files (`.prompt.md`)  
**Topics**:
- Frontmatter fields (description, name, agent, tools)
- File naming and placement
- Body structure with logical sections
- Input and context handling
- Tool permissions
- Output definition and validation

**Use when**: Creating or updating prompt files

## How Instructions Are Used

### For Code Authors
1. Check [engineering.instructions.md](engineering.instructions.md) for general principles
2. For your language, check language-specific file (e.g., python.instructions.md)
3. Follow project patterns shown in examples

### For Agent Creators
1. Check [agents.instructions.md](agents.instructions.md) for structure
2. Review existing agents in `.github/agents/` for examples
3. Apply [engineering.instructions.md](engineering.instructions.md) principles to agent behavior

### For Skill Creators
1. Check [agent-skills.instructions.md](agent-skills.instructions.md) for structure
2. Review existing skills in `.github/skills/` for examples
3. Ensure description follows WHAT/WHEN/KEYWORDS pattern for discoverability

### For Prompt Creators
1. Check [prompt.instructions.md](prompt.instructions.md) for structure
2. Reference existing prompts for patterns
3. Validate against the checklist before publishing

## Key Principles Applied Across All Files

- **Single source of truth**: General principles in engineering.instructions.md
- **Clear references**: Links guide you to related guidance
- **Language clarity**: Specific guidance separated by language/context
- **Actionable**: Each file includes examples and checklists
- **Maintainable**: Centralized principles reduce duplication

## Maintenance Notes

All instruction files follow the applyTo pattern for scope identification:
- `**/*.py` - Python files
- `**/*.agent.md` - Custom agent files
- `**/.github/skills/**/SKILL.md` - Skill files
- `**/*.prompt.md` - Prompt files
- `**` - General principles (engineering.instructions.md)

When updating principles, ensure consistency across referenced files.

