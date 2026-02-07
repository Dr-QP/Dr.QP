---
name: code-review-standards
description: Apply code review standards and write high-quality pull request descriptions. Use when asked to review code, write PR descriptions, create commit messages, or submit pull requests. Enforces imperative language, comprehensive descriptions, PR best practices, and clean code principles.

---

# Code Review Standards

Write high-quality pull request descriptions and conduct effective code reviews using established best practices. For detailed engineering principles and standards, refer to [shared engineering guidelines](../../instructions/engineering.instructions.md).

## When to Use This Skill

- Write PR descriptions for GitHub
- Create comprehensive commit messages
- Review code for quality and standards compliance
- Ensure PR documentation is complete
- Apply clean code principles to reviews
- Evaluate code for maintainability
- Provide constructive feedback on changes
- Validate against project coding standards

## Prerequisites

- Understanding of project coding standards (see AGENTS.md)
- Knowledge of changed files and their purposes
- Context about what the changes accomplish
- Understanding of why specific design decisions were made
- Familiarity with the codebase being reviewed

## PR Description Best Practices

### Golden Rules

1. **Use imperative mood**: "Fix", "Add", "Refactor" (not "Fixes", "Added", "Refactoring")
2. **Write for clarity**: Assume reader knows nothing about your changes
3. **Be comprehensive**: Include what changed and why, not just what
4. **Avoid tables**: Never use per-file changes tables or line counts
5. **No marketing**: Never add Copilot ads or AI tool information
6. **No invisible content**: Never use HTML comments or hidden Unicode
7. **Treat as git commit**: Apply highest standards; this is permanent project history

## Step-by-Step Workflows

### Workflow 1: Write PR Description (Recommended Template)

Create a comprehensive pull request description.

1. Start with concise summary (1-2 lines):
   ```
   Add timeout handling to serial communication driver
   ```

2. Add "What changed" section describing modifications:
   ```markdown
   ## What Changed

   - Implement exponential backoff retry logic for failed reads
   - Add configurable timeout parameters to package.xml
   - Remove deprecated synchronous communication patterns
   - Add comprehensive timeout error messages
   ```

3. Add "Why" section explaining motivation:
   ```markdown
   ## Why

   Serial device timeouts cause ROS nodes to block indefinitely,
   preventing graceful shutdown. This change allows nodes to recover
   and log meaningful errors for debugging.
   ```

4. Add "How to test" section with verification steps:
   ```markdown
   ## How to Test

   1. Run drqp_serial tests: `python3 -m colcon test --packages-select drqp_serial`
   2. Verify coverage increased: Check build/drqp_serial/coverage.info
   3. Manual test: Connect USB device, disconnect mid-communication,
      verify timeout and recovery
   ```

5. Add "Related issues" if applicable:
   ```markdown
   ## Related Issues

   Closes #45
   Related to #42, #43
   ```

6. Add "Breaking changes" if any:
   ```markdown
   ## Breaking Changes

   - Changed `serial_read()` timeout parameter from milliseconds to seconds
   - Removed `sync_read()` function (use async variant instead)
   ```

### Workflow 2: Review Code for Quality Standards

Evaluate code changes against project standards.

**Check these aspects:**

1. **Readability and Clarity**
   - Variable/function names are clear and descriptive
   - Code is understandable without excessive comments
   - Complex logic is broken into smaller functions
   - Nesting is not excessive (max 3-4 levels)

2. **Maintainability**
   - Functions have single, clear responsibility
   - Code avoids duplication (DRY principle)
   - Dependencies on external libraries are documented
   - Design decisions are explained in comments

3. **Error Handling**
   - All error cases are handled appropriately
   - Exception messages are clear and actionable
   - Edge cases are considered and tested
   - No silent failures

4. **Testing**
   - Code changes include corresponding tests
   - Tests cover happy path and error cases
   - Coverage is maintained or improved
   - Test names clearly describe what they verify

5. **Standards Compliance**
   - Follows language-specific conventions (PEP 8 for Python, clang-format for C++)
   - Consistent with codebase patterns
   - No deprecated patterns or libraries
   - Aligns with project architecture

### Workflow 3: Provide Constructive Code Review Feedback

Write helpful, actionable review comments.

**Good feedback example:**
```
The mutex lock here could be a deadlock risk if `process_data()`
throws an exception. Consider using RAII pattern or try/finally
to ensure lock is always released.
```

**Poor feedback example:**
```
This is bad. Fix it.
```

**Guidelines:**

1. **Be specific**: Point to exact lines and explain why it's an issue
2. **Suggest solutions**: Provide code examples when helpful
3. **Use neutral language**: Avoid judgmental tone
4. **Consider context**: Ask questions if you don't understand intent
5. **Praise good code**: Acknowledge well-written sections
6. **Focus on impact**: Explain how the issue affects functionality/maintenance

**Common feedback categories:**

```
## Clarity Issues
- "Variable name X is ambiguous; consider Y for clarity"
- "This logic could be extracted to method Z for reuse"

## Potential Bugs
- "This could fail if Z is null; add guard clause"
- "Race condition possible here if called from multiple threads"

## Performance
- "O(n²) loop could be optimized to O(n) using a set"
- "String concatenation in loop creates many allocations"

## Testing
- "This error case isn't tested; add test_X"
- "Coverage would be higher if we test the else branch"

## Architecture
- "This couples module A to module B unnecessarily"
- "Consider using pattern X instead for better separation"
```

### Workflow 4: Validate Against Clean Code Principles

Check code against established quality standards.

| Principle | Check | Example |
|-----------|-------|---------|
| **Single Responsibility** | Does function do one thing? | Function calculates AND formats AND logs → split into 3 |
| **Descriptive Names** | Are names self-documenting? | `x`, `temp` → `milliseconds_elapsed`, `retry_count` |
| **Small Functions** | Can function be understood in 2 minutes? | 200-line function → split into 5 functions |
| **Error Handling** | Are all error paths explicit? | Silent failures → throw with context |
| **Tests** | Are tests clear and comprehensive? | Single assertion per test, clear test names |
| **Comments** | Are comments explaining "why" not "what"? | `// increment counter` → `// retry count to handle transient failures` |
| **DRY** | Is code duplicated? | Same logic in 3 places → extract to function |
| **Dependencies** | Are dependencies minimal and explicit? | Using everything from library → import only what's used |

### Workflow 5: Handle Review Feedback in PR Updates

Update PR based on code review feedback.

1. Make requested code changes
2. Commit with clear message:
   ```bash
   git commit -m "Address review feedback: simplify error handling logic"
   ```

3. Update PR description if scope changed:
   - Add new accomplishments to "What Changed"
   - Update "How to Test" if testing changes
   - Document why feedback was accepted or rejected

4. Reply to comments:
   - Acknowledge the feedback
   - Explain changes made
   - Link to specific commits

5. Request re-review when ready

## Commit Message Standards

### Template

```
<type>: <subject>

<body>

<footer>
```

### Example

```
refactor: extract timeout logic from serial_read

Extract exponential backoff retry logic into separate function
to improve reusability and testability. This prepares for future
timeout configuration in package.xml.

Relates to #45
```

### Guidelines

- **Type**: fix, feat, refactor, docs, test, chore, perf
- **Subject**: Imperative, lowercase, no period, max 50 chars
- **Body**: Explain what and why (not what code does), wrap at 72 chars
- **Footer**: Reference related issues

## Troubleshooting

| Issue | Solution |
|-------|----------|
| PR description too technical | Add "Why" section explaining user impact |
| Feedback seems nitpicky | Consider if it genuinely improves maintainability |
| Author defensive about feedback | Keep tone neutral; focus on code, not person |
| Conflicting feedback from reviewers | Discuss in thread; find consensus or escalate |
| Changes seem unnecessary | Ask for clarification of impact/rationale |
| Too many comments per review | Prioritize critical issues; discuss style separately |

## Review Checklist

Before approving a PR, verify:

- [ ] PR description is comprehensive and uses imperative mood
- [ ] Code follows project coding standards
- [ ] All error cases are handled
- [ ] Tests are included and comprehensive
- [ ] Test coverage maintained or improved
- [ ] No duplication or code smell issues
- [ ] Design aligns with project architecture
- [ ] Comments explain "why" not "what"
- [ ] No breaking changes without documentation
- [ ] Related issues properly referenced

## References

- See PR template guide for detailed examples
- Use a Pull Request Template for GitHub auto-fill of standard sections
- Use a code review checklist for systematic reviews
- Review clean code principles and standards regularly

