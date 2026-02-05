# Skill Validators

This package provides validation for Agent Skills (SKILL.md files).

## Architecture

The validation system uses a hybrid approach:

- **Pydantic Models** ([models.py](models.py)): For schema-based validation of frontmatter and structure
  - `SkillFrontmatter`: Validates YAML frontmatter (name, description, license)
  - `SkillStructure`: Validates markdown structure and required sections
  
- **Custom Validators**: For validations that require context beyond single files
  - `UniquenessValidator`: Cross-skill uniqueness checks
  - `CrossReferenceValidator`: File reference validation

## Migration from Custom Validators

The following validators have been **replaced by Pydantic models**:

- `frontmatter_validator.py` → `models.SkillFrontmatter`
- `description_validator.py` → `models.SkillFrontmatter` (description validation)
- `structure_validator.py` → `models.SkillStructure`

### Benefits of Pydantic

- **Type-safe**: Leverages Python type hints
- **Better errors**: Clear, structured validation errors
- **Less code**: 52% reduction in validation logic
- **Industry standard**: Used by 360M+ downloads/month
- **Maintainable**: Declarative schema definitions

## Code Reduction

| Component | Before (lines) | After (lines) | Reduction |
|-----------|----------------|---------------|-----------|
| Frontmatter validation | 113 | Part of 182 | ~52% |
| Description validation | 141 | Part of 182 | ~52% |
| Structure validation | 127 | Part of 182 | ~52% |
| **Total** | **381** | **182** | **52%** |

Custom validators (uniqueness, xref) remain unchanged as they require cross-file context.

## Usage

```python
from skill_validators.models import SkillFrontmatter, SkillStructure
from pydantic import ValidationError
import frontmatter

# Parse skill file
post = frontmatter.load('SKILL.md')

# Validate frontmatter
try:
    skill = SkillFrontmatter(**post.metadata)
except ValidationError as e:
    print(e.errors())

# Validate structure
try:
    structure = SkillStructure(body=post.content)
    warnings = structure.get_warnings()  # Optional warnings
except ValidationError as e:
    print(e.errors())
```
