# Skill Validation Implementation Summary

This document summarizes the skill validation infrastructure added to the Dr.QP repository.

## Overview

Added comprehensive validation scripts to automatically check Agent Skills for GitHub Copilot against established patterns and best practices, ensuring skills can be effectively discovered and used.

## Components Implemented

### 1. Validation Modules (`scripts/skill_validators/`)

Modular validation system with five specialized validators:

- **`frontmatter_validator.py`**: Validates YAML frontmatter (name, description, license)
- **`description_validator.py`**: Validates WHAT/WHEN/KEYWORDS pattern for discovery
- **`structure_validator.py`**: Validates required sections and content structure
- **`uniqueness_validator.py`**: Detects duplicate names and keyword conflicts
- **`xref_validator.py`**: Validates cross-references and internal links

### 2. Main Validation Script (`scripts/validate-skills.py`)

CLI tool for running validations:

```bash
# Validate all skills
python3 scripts/validate-skills.py

# Validate specific skill
python3 scripts/validate-skills.py .github/skills/my-skill/SKILL.md

# CI mode (exit 1 on errors)
python3 scripts/validate-skills.py --ci

# Show warnings and recommendations
python3 scripts/validate-skills.py --recommend
```

**Features:**
- Single file or batch validation
- Colorized output with clear error messages
- CI mode for automated pipelines
- Warning/recommendation mode for best practices

### 3. Test Suite (`scripts/test/`)

Comprehensive test coverage:

- **17 unit tests** covering all validation rules
- **End-to-end tests** with good/bad skill fixtures
- **100% pass rate** on all validators

**Test fixtures:**
- `scripts/test/fixtures/good-skill/SKILL.md` - Example of compliant skill
- `scripts/test/fixtures/bad-skill/SKILL.md` - Example with multiple violations

### 4. CI/CD Integration

#### GitHub Actions Workflow (`.github/workflows/validate-skills.yml`)

Automatically runs on:
- Push to main/develop (when skill files change)
- Pull requests (when skill files change)
- Manual workflow dispatch

Validates all skills and runs test suite on every change.

#### Pre-commit Hook (`.pre-commit-config.yaml`)

Added `validate-skills` hook that:
- Runs automatically before commits
- Only triggers on SKILL.md file changes
- Validates in CI mode (fails commit on errors)

### 5. Documentation

#### Skills README (`.github/skills/README.md`)

Comprehensive guide covering:
- What Agent Skills are
- Complete validation rules
- Usage examples
- Integration points (CI, pre-commit, VS Code)
- Skill creation workflow

#### VS Code Tasks (`.vscode/tasks.json`)

Added three tasks (local configuration):
1. **Validate Skills** - Run with recommendations
2. **Validate Skills (CI Mode)** - Strict validation
3. **Test Skill Validation** - Run test suite

*Note: tasks.json is gitignored but documented for local setup*

## Validation Rules Summary

### Critical Rules (Errors)

1. **Frontmatter**:
   - name: lowercase-hyphenated, max 64 chars
   - description: 100-1024 chars
   - Both fields required

2. **Description Pattern**:
   - Must include WHAT (capabilities)
   - Must include WHEN (trigger phrases)
   - Minimum 5 unique keywords
   - No vague terms (helpers, utilities, stuff, etc.)

3. **Structure**:
   - Required: "When to Use This Skill" section
   - Required: "Prerequisites" section

4. **Uniqueness**:
   - No duplicate skill names

5. **Cross-references**:
   - All internal links must resolve

### Recommended Practices (Warnings)

- Include license field
- Add Troubleshooting section (with table)
- Include at least 2 workflow sections
- Avoid overly broad keywords
- Keep body under 500 lines
- List skill in README.md

## Usage Examples

### For Developers

**Before committing a new skill:**
```bash
python3 scripts/validate-skills.py .github/skills/my-new-skill/ --recommend
```

**Run tests:**
```bash
python3 scripts/test/test_skill_validation.py -v
```

### For CI/CD

**In GitHub Actions:**
```yaml
- name: Validate skills
  run: python3 scripts/validate-skills.py --ci --recommend
```

**In pre-commit:**
```yaml
- id: validate-skills
  entry: python3 scripts/validate-skills.py --ci
  files: '(\.github/skills/.*SKILL\.md|\.claude/skills/.*SKILL\.md)$'
```

## Example Output

**Good skill:**
```
Validating 1 skill(s)...

✓ .github/skills/example-good-skill/SKILL.md

Summary: 1 passed, 0 failed
```

**Bad skill:**
```
Validating 1 skill(s)...

✗ .github/skills/bad-skill/SKILL.md
  ✗ 'name' must be lowercase with hyphens (e.g., 'my-skill-name'), got: 'Bad_Skill_Name' [frontmatter]
  ✗ 'description' is too short (12 chars). Minimum 100 characters required [frontmatter]
  ✗ Description contains vague terms that hinder discovery: helpers [description]
  ✗ Description missing WHEN guidance [description]
  ✗ Missing required section: 'When To Use This Skill' [structure]
  ✗ Missing required section: 'Prerequisites' [structure]

Summary: 0 passed, 1 failed
```

## Files Changed

### New Files
- `scripts/skill_validators/__init__.py`
- `scripts/skill_validators/frontmatter_validator.py`
- `scripts/skill_validators/description_validator.py`
- `scripts/skill_validators/structure_validator.py`
- `scripts/skill_validators/uniqueness_validator.py`
- `scripts/skill_validators/xref_validator.py`
- `scripts/validate-skills.py`
- `scripts/test/test_skill_validation.py`
- `scripts/test/fixtures/good-skill/SKILL.md`
- `scripts/test/fixtures/bad-skill/SKILL.md`
- `.github/workflows/validate-skills.yml`
- `.github/skills/README.md`

### Modified Files
- `.pre-commit-config.yaml` (added validate-skills hook)

### Local Files (not tracked)
- `.vscode/tasks.json` (documented for developers to add locally)

## Testing Results

All 17 tests passing:

- ✅ Frontmatter validation (6 tests)
- ✅ Description pattern validation (4 tests)
- ✅ Structure validation (3 tests)
- ✅ Uniqueness validation (2 tests)
- ✅ End-to-end validation (2 tests)

## Dependencies

**Required:**
- Python 3.12+
- PyYAML (for frontmatter parsing)

**No additional dependencies** - uses Python standard library for all validators.

## Future Enhancements

Potential improvements not implemented in this PR:

1. **Auto-fix mode**: Automatically fix common issues (e.g., convert name to lowercase)
2. **Detailed reports**: JSON/HTML output for CI dashboards
3. **Performance metrics**: Track validation time for large skill libraries
4. **Keyword analyzer**: Suggest additional keywords based on body content
5. **Template generator**: CLI command to scaffold new skills with valid structure

## Related Issues

- Addresses issue: "Add validation scripts for skill description patterns"
- Related to Dr-QP/Dr.QP#266 (agent and skills infrastructure)
- Prevents: Skills never activating, false activations, conflicting skills

## References

- [Agent Skills Specification](https://agentskills.io/)
- [Agent Skills Guidelines](../.github/instructions/agent-skills.instructions.md)
- [VS Code Agent Skills Documentation](https://code.visualstudio.com/docs/copilot/customization/agent-skills)
