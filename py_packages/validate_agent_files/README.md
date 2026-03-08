# validate_agent_files

CLI tools for validating DRQP skills, agent files, and prompt files.

## Installation

```bash
pip install -e .
```

## Usage

```bash
validate_agent_files                     # Validate skills, agents, and prompts under .
validate_agent_files .github             # Validate canonical repo customizations
validate_agent_files --kind skills       # Validate only skill files
validate_agent_files --kind agents       # Validate only agent files
validate_agent_files --kind prompts      # Validate only prompt files
validate_agent_files --recommend         # Show recommendations
validate_agent_files --ci                # CI mode (nonzero exit on errors)
```

## Notes

- Skill frontmatter validation is delegated to `skills-ref`.
- Local validation still checks repository-specific rules such as duplicate skill names,
  cross-references, agent handoffs, and prompt `#file:` references.
