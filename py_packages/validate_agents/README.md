# validate_agents

CLI tools for validating DRQP skills, agent files, and prompt files.

## Installation

```bash
pip install -e .
```

## Usage

```bash
validate_customizations                  # Validate skills, agents, and prompts under .
validate_customizations .github          # Validate canonical repo customizations
validate_customizations --kind agents    # Validate only agent files
validate_customizations --kind prompts   # Validate only prompt files

validate_skills                          # Backward-compatible skills-only validation
validate_skills path/to/skills/          # Validate specific skill directory
validate_skills --recommend              # Show recommendations
validate_skills --ci                     # CI mode (nonzero exit on errors)
```

## Notes

- Skill frontmatter validation is delegated to `skills-ref`.
- Local validation still checks repository-specific rules such as duplicate skill names,
  cross-references, agent handoffs, and prompt `#file:` references.
