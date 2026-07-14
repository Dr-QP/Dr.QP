# validate_agent_files

CLI tools for validating DRQP skills, agent files, and prompt files.

## Installation

```bash
pip install -e .
```

## Usage

```bash
validate_agent_files                     # Validate skills, agents, and prompts under .
validate_agent_files .claude             # Validate canonical repo customizations
validate_agent_files .claude/skills .claude/agents  # Validate multiple paths
validate_agent_files --kind skills       # Validate only skill files
validate_agent_files --kind agents       # Validate only agent files
validate_agent_files --kind prompts      # Validate only prompt files
validate_agent_files --recommend         # Show recommendations
validate_agent_files --ci                # CI mode (nonzero exit on errors)
```

## Notes

- `skills-ref` is a required dependency and the primary validator for skill files.
- Local validation still checks repository-specific rules such as duplicate skill names,
  cross-references, agent handoffs, prompt `#file:` references, and Codex trampoline sync
  (each `.claude/agents/<stem>.agent.md` must have a `.codex/agents/<stem>.md` trampoline
  with a matching `name` and `description`, and no orphan trampolines).
