---
name: code-review-standards
description: Apply code review standards and write high-quality pull request descriptions. Use when asked to review code, write PR descriptions, create commit messages, or submit pull requests. Enforces imperative language, comprehensive descriptions, PR best practices, and clean code principles.

---

# Code Review Standards

Write high-quality pull request descriptions and conduct effective code reviews using established best practices. Use [shared engineering guidelines](../../instructions/engineering.instructions.md) as the source of truth for generic code quality, testing, error handling, and clean-code expectations.

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

## Scope

This skill covers review-specific workflow and PR-writing structure. Do not restate broad engineering rules from the shared instructions unless the review needs a tighter, task-specific requirement.

## PR Description Workflow

### Golden Rules

1. **Use imperative mood**: "Fix", "Add", "Refactor" (not "Fixes", "Added", "Refactoring")
2. **Write for clarity**: Assume reader knows nothing about your changes
3. **Be comprehensive**: Include what changed and why, not just what
4. **Avoid tables**: Never use per-file changes tables or line counts
5. **No marketing**: Never add Copilot ads or AI tool information
6. **No invisible content**: Never use HTML comments or hidden Unicode
7. **Treat as git commit**: Apply highest standards; this is permanent project history

### Recommended Template

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

## Review Workflow

Evaluate code changes against the shared engineering standards, then focus your review on findings, evidence, and impact.

### What To Check

1. Correctness and behavioral regressions
2. Missing or weak tests for changed behavior
3. Maintainability risks that materially raise future cost
4. Security or performance issues with concrete impact
5. Standards violations that conflict with the shared engineering guidelines

### How To Write Findings

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

### Common Finding Categories

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

## Responding To Review Feedback

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

## References

- [Shared engineering guidelines](../../instructions/engineering.instructions.md)
- [Generate pull request description](../generate-pr-description/)

