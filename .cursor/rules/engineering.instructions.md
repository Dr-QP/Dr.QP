---
description: 'Shared engineering principles and best practices for code quality, testing, and design'
globs:
  - '**/*'
---

> Cursor mirror of .github/instructions/engineering.instructions.md. Keep in sync.

# Shared Engineering Principles

Unified engineering standards and best practices that apply across all code, agents, skills, and instructions in this workspace.

## Core Engineering Principles

### Code Quality & Clarity

- **Always prioritize readability and clarity** -- Code should be easy to understand without excessive comments
- Write for the reader, not the compiler/interpreter
- Name variables, functions, and classes descriptively
- Use consistent naming conventions matching language standards
- Break complex functions into smaller, testable units
- Avoid deep nesting; refactor into helper functions

### Design & Architecture

- **Apply SOLID Principles**:
  - Single Responsibility Principle (SRP)
  - Open/Closed Principle
  - Liskov Substitution Principle
  - Interface Segregation Principle
  - Dependency Inversion Principle

- **Apply Clean Code Principles** (Robert C. Martin):
  - Meaningful names
  - Functions should do one thing
  - DRY (Don't Repeat Yourself)
  - KISS (Keep It Simple, Stupid)
  - YAGNI (You Aren't Gonna Need It)

- **Use Design Patterns** where appropriate:
  - Gang of Four patterns for recurring design problems
  - Avoid over-engineering; use patterns pragmatically

### Implementation Practices

- **Always handle edge cases**:
  - Empty/null inputs
  - Invalid data types
  - Large datasets
  - Boundary conditions
  - Error states

- **Write clear exception handling**:
  - Use specific exception types
  - Include meaningful error messages
  - Document why specific exceptions are caught/thrown
  - Never silently ignore errors

- **Prefer libraries and frameworks** over custom implementations:
  - Use well-maintained, battle-tested libraries
  - Avoid reinventing the wheel
  - Document external dependencies and their purpose
  - Keep dependencies up-to-date

- **Include documentation and comments**:
  - Explain *why* decisions were made, not *what* the code does
  - Document algorithm approaches for complex code
  - Use comments sparingly for non-obvious logic
  - Keep comments synchronized with code

### Testing

- **Write tests for critical paths** and business logic
- Use Test-Driven Development (TDD) when appropriate:
  - Write failing tests first (Red)
  - Implement minimal code to pass (Green)
  - Refactor to improve design (Refactor)

- **Include edge case tests**:
  - Document expected behavior in test comments
  - Test boundary conditions
  - Test error scenarios

- **Test pyramid approach**:
  - Many unit tests (fast, isolated)
  - Fewer integration tests (verify components work together)
  - Minimal end-to-end tests (validate complete workflows)

- **Write deterministic tests**:
  - Same input always produces same output
  - No flaky tests that pass/fail randomly
  - Clear test names that describe what is being tested

## Language-Specific Conventions

### C++ Guidelines

- Follow the C++ Core Guidelines and ISO C++ Standard
- Use modern C++ features (C++17 or later):
  - RAII (Resource Acquisition Is Initialization) for resource management
  - Value semantics by default
  - Smart pointers instead of raw pointers
  - Standard library containers and algorithms

- Prefer RAII over manual resource management
- Make ownership explicit in API design
- Use static analysis and sanitizers during development
- Focus on correctness first, then optimize with evidence

### Python Guidelines

- Follow **PEP 8** style guide:
  - 4 spaces per indentation level
  - Lines <= 79 characters
  - Meaningful variable and function names

- Use type hints (PEP 484):
  - Annotate function parameters and return types
  - Use `typing` module for complex types
  - Run type checkers (mypy) as part of CI

- Write docstrings following **PEP 257**:
  - One-line summary for simple functions
  - Multi-line for complex functions with parameter/return documentation
  - Use triple-quoted strings

- Example:
  ```python
  def calculate_area(radius: float) -> float:
      """
      Calculate the area of a circle given the radius.
      
      Args:
          radius: The radius of the circle in units
          
      Returns:
          The area in square units (pi * r^2)
      """
      import math
      return math.pi * radius ** 2
  ```

### ROS 2 Specifics

- Follow ROS 2 naming conventions for packages and nodes
- Document package dependencies in `package.xml`
- Use consistent naming for topics, services, and actions
- Include launch files for complex multi-node systems
- Write integration tests for node interactions
- Document parameter defaults and constraints

## Working with Technical Debt

When technical debt is identified or incurred:

1. **Document the debt explicitly**:
   - Explain why the shortcut was necessary
   - Note the consequences if left unaddressed
   - Suggest remediation approach

2. **Track remediation**:
   - Create GitHub Issues for debt items
   - Include impact assessment in issue description
   - Link related issues and dependencies

3. **Prioritize strategically**:
   - Address critical path issues first
   - Fix security/stability issues immediately
   - Schedule other improvements for planned refactoring

4. **Prevent accumulation**:
   - Review code for emerging debt patterns
   - Refactor small amounts frequently
   - Use dedicated refactoring sprints for large debt

## Code Review Standards

When reviewing code:

- **Check for correctness**: Does it do what it's supposed to do?
- **Verify test coverage**: Are edge cases tested?
- **Assess maintainability**: Will future developers understand it?
- **Review security**: Are there potential vulnerabilities?
- **Check performance**: Are there obvious inefficiencies?
- **Verify standards**: Does it follow project conventions?

**Provide constructive feedback**:
- Explain *why* a change is needed
- Suggest alternatives if rejecting an approach
- Acknowledge good work and learning
- Be respectful and collaborative

## Quality Assurance Checklist

Before completing work:

- [ ] Code follows language conventions and project standards
- [ ] Edge cases are handled appropriately
- [ ] Error handling is clear and meaningful
- [ ] Tests are written for critical paths
- [ ] Documentation is accurate and up-to-date
- [ ] No unnecessary complexity or over-engineering
- [ ] External dependencies are documented
- [ ] Potential technical debt is noted
- [ ] Security considerations are addressed
- [ ] Performance is acceptable (with evidence if optimized)

## Continuous Learning

- Stay current with language/framework updates
- Review and learn from pull request reviews
- Participate in code reviews to share knowledge
- Refactor legacy code to apply new patterns
- Share knowledge through documentation and mentoring
