# Skills

Agent Skills for the Dr.QP project. These skills provide specialized capabilities that enhance AI agent functionality.

## Available Skills

### find-test-files

**Description:** Locate test files in the workspace by searching for common test file patterns and conventions.

**When to use:** Use when asked to "find tests", "locate test files", "list all tests", "discover test suites", or when working with TDD workflows, test analysis, or test coverage tasks.

**Supports:**
- Python test patterns (pytest, unittest)
- C++ test patterns (Google Test, Catch2)  
- TypeScript/JavaScript test patterns (Jest, Vitest, Mocha, Jasmine)
- ROS 2 test patterns

**Key features:**
- Comprehensive test file pattern matching
- Language-specific test discovery strategies
- Integration with TDD workflows (Red, Green, Refactor phases)
- Content-based test file discovery
- Package/module-scoped test searches

**File:** [find-test-files/SKILL.md](find-test-files/SKILL.md)

## How to Use Skills

Skills are automatically discovered by GitHub Copilot when they are placed in the `.github/skills/` directory. The agent will load the skill instructions when your request matches the skill's description.

## Creating New Skills

To create a new skill:

1. Create a new directory under `.github/skills/` with a lowercase, hyphenated name
2. Create a `SKILL.md` file with proper YAML frontmatter:
   ```yaml
   ---
   name: my-skill-name
   description: 'Clear description of what the skill does and when to use it.'
   ---
   ```
3. Add detailed instructions in the body of `SKILL.md`
4. Optionally add bundled assets (scripts, references, templates) in subdirectories

For more information, see the [Agent Skills specification](https://agentskills.io/specification).
