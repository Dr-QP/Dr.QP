# validate_agents

CLI tools for validating DRQP agents and their skills.

## Installation

```bash
pip install -e .
```

## Usage

```bash
validate_skills                    # Validate .github/skills
validate_skills path/to/skills/     # Validate specific directory
validate_skills --recommend         # Show recommendations
validate_skills --ci                # CI mode (nonzero exit on errors)
```
