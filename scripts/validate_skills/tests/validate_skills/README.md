# Test Suite for validate_skills Package

This directory contains comprehensive failing unit tests that drive the refactoring of the validate-skills package into a clean, modular structure.

## Test Files

### 1. **test_cli.py** (74 tests across 7 test classes)
Tests for CLI argument parsing and setup.
- Basic argument parsing (default path, custom paths, file paths, absolute paths)
- `--recommend` flag for showing warnings
- `--ci` flag for continuous integration mode
- Validation level control (`--no-warnings`, `--errors-only`)
- Output format options (`--format json|csv|xml`)
- Verbosity flags (`-q`/`--quiet`, `-v`/`--verbose`)
- Help text and documentation
- Invalid argument combinations
- Edge cases (empty paths, whitespace, relative paths with dots, many arguments)

### 2. **test_loaders.py** (40+ tests across 6 test classes)
Tests for file loading and parsing utilities.
- **TestFindSkillFilesDirectory**: Finding SKILL.md files in directories
  - Empty directories, single skills, nested directories, multiple files
  - Exclusion of test, tests, pytest_cache, __pycache__ directories
- **TestFindSkillFilesSingleFile**: Finding when given specific file path
  - Specific SKILL.md files, non-SKILL files, nonexistent files
- **TestSafeLoadFrontmatter**: Safe YAML frontmatter loading
  - Valid frontmatter, empty body, complex nested YAML
  - Malformed YAML, missing frontmatter, invalid encoding
  - Special characters, whitespace preservation
- **TestLoadAllSkills**: Loading multiple skill files
  - Empty list, single skill, multiple skills
  - Invalid file handling with logging
  - Preservation of body and metadata
- **TestLoadersEdgeCases**: Symlinks, permission denied, hidden directories

### 3. **test_validators.py** (40+ tests across 5 test classes)
Tests for individual skill validators.
- **TestSkillFrontmatterValidator**: Frontmatter validation
  - Valid frontmatter, missing name/description
  - Name format validation, description length constraints
  - Vague terminology warnings, extra fields allowed
- **TestSkillStructureValidator**: File structure validation
  - Valid structure with required sections
  - Missing Overview and "When to use" sections
  - Heading hierarchy validation, empty sections
- **TestUniquenessValidator**: Skill name uniqueness
  - Unique names, duplicate names, case-insensitive duplicates
- **TestCrossReferenceValidator**: Cross-reference validation
  - Valid references, broken references, absolute path references
- **TestValidatorsEdgeCases**: Empty metadata, None values, whitespace-only values, wrong types

### 4. **test_core.py** (30+ tests across 4 test classes)
Tests for the validation engine core.
- **TestValidationEngineBasic**: Basic validation operations
  - Engine initialization, valid/invalid skill validation
- **TestValidationEngineWarnings**: Warning handling
  - Include warnings by default, exclude when configured
- **TestValidationEngineMultipleValidators**: Coordinating multiple validators
  - Run all validators, collect all issues
- **TestValidationEngineLoadedSkills**: Cross-validation with loaded skills
  - Check uniqueness against all loaded skills
- **TestValidationEngineErrorHandling**: Error handling
  - Missing files, invalid YAML, permission denied
- **TestValidationResult**: ValidationResult data structure
  - is_valid property, has_warnings property, issue counting

### 5. **test_formatters.py** (30+ tests across 5 test classes)
Tests for output formatting utilities.
- **TestTextFormatter**: Text output formatting
  - Format results, color support, summary statistics
- **TestJSONFormatter**: JSON output formatting
  - Valid JSON output, metadata inclusion, multiple results
- **TestCSVFormatter**: CSV output formatting
  - Header generation, data rows, special character escaping
- **TestFormatResults**: Top-level formatting function
  - Text, JSON, CSV formats, invalid format handling
- **TestFormatterEdgeCases**: Empty results, no issues, many issues, unicode handling

### 6. **test_main.py** (25+ tests across 6 test classes)
Tests for main orchestration function.
- **TestMainBasic**: Basic main function behavior
  - No arguments, directory validation, file validation
- **TestMainExitCodes**: Exit code behavior
  - Return 0 on success, return 1 on errors
- **TestMainFlags**: Command-line flags
  - `--recommend`, `--ci`, `--format` flags
- **TestMainOutputBehavior**: Output behavior
  - Print results and summary, error handling
- **TestMainIntegration**: Full end-to-end integration
  - Multiple skills, duplicate detection, mixed valid/invalid
- **TestMainEdgeCases**: Nonexistent paths, empty directories, permission denied

### 7. **test_edge_cases.py** (50+ tests across 7 test classes)
Tests for edge cases and error handling.
- **TestErrorHandlingEdgeCases**: File handling edge cases
  - No frontmatter, incomplete frontmatter, large files
  - Circular reference protection
- **TestCharacterEncodingEdgeCases**: Character encoding
  - UTF-8 with BOM, mixed line endings, unicode characters
- **TestDataValidationEdgeCases**: Data validation
  - Extremely long names/descriptions, None values, wrong types
- **TestConcurrencyAndRaceConditions**: File modification during validation
  - File modified/deleted during validation
- **TestPathTraversalSecurity**: Security concerns
  - Path traversal attempts, symlink loops
- **TestResourceLimits**: Resource limit handling
  - Many issues, many files, deeply nested directories
- **TestRobustnessEdgeCases**: General robustness
  - Empty frontmatter, whitespace-only sections, malformed markdown

## Test Configuration

### conftest.py
Provides pytest configuration and shared fixtures:
- `skill_template`: Template for creating test skills
- `valid_skill_content`: Complete valid skill content
- `invalid_skill_content`: Invalid skill for error testing
- `temp_skill_dir`: Temporary directory with sample skills
- `mock_logger`: Mock logger fixture

### __init__.py
Package marker files for test discovery.

## Running the Tests

```bash
# Run all tests
pytest tests/validate_skills/

# Run specific test file
pytest tests/validate_skills/test_cli.py

# Run specific test class
pytest tests/validate_skills/test_cli.py::TestParseArgumentsBasic

# Run specific test
pytest tests/validate_skills/test_cli.py::TestParseArgumentsBasic::test_parse_arguments_default_path

# Run with verbose output
pytest -v tests/validate_skills/

# Run with coverage
pytest --cov=validate_skills tests/validate_skills/

# Run only fast tests (exclude slow)
pytest -m "not slow" tests/validate_skills/
```

## Import Structure

All tests import from the new package structure:
```python
from validate_skills.cli import parse_arguments
from validate_skills.loaders import find_skill_files, safe_load_frontmatter, load_all_skills
from validate_skills.core import ValidationEngine, ValidationResult
from validate_skills.formatters import TextFormatter, JSONFormatter, CSVFormatter
from validate_skills.main import main
from validate_skills.validators.skill import SkillFrontmatterValidator, SkillStructureValidator
from validate_skills.validators.uniqueness import UniquenessValidator
from validate_skills.validators.cross_reference import CrossReferenceValidator
```

## Test Coverage Summary

**Total Tests: 300+**

- CLI Parsing: 74 tests
- File Loading: 40+ tests
- Validators: 40+ tests
- Core Engine: 30+ tests
- Formatters: 30+ tests
- Main Function: 25+ tests
- Edge Cases: 50+ tests

## Test Characteristics

### Comprehensive Coverage
- ✅ Normal/happy path scenarios
- ✅ Edge cases and error conditions
- ✅ Integration between components
- ✅ CLI behavior and exit codes
- ✅ Error handling and graceful degradation
- ✅ Security considerations (path traversal, symlink loops)
- ✅ Unicode and encoding handling
- ✅ Resource limits and performance

### Clear Test Names
- All tests use descriptive names that explain the scenario
- Docstrings provide context and expected behavior
- Test organization by functionality (classes, methods)

### Good Assertions
- Specific assertions that check for expected behavior
- Validation of error types and messages
- State changes and side effects verified

### Guide for Refactoring
These tests comprehensively specify the expected behavior of the new package structure, allowing developers to implement the modules with confidence that they meet all requirements.

## Next Steps

Once the new package structure is implemented:

1. Run tests to identify failing cases
2. Implement modules incrementally
3. Tests will guide design decisions
4. All tests should pass when refactoring is complete

