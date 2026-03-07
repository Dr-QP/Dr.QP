---
name: pr-feedback-resolution
description: Systematic workflow for addressing PR feedback, CI failures, security findings, and coverage gaps. Use when resolving code review comments, fixing CI issues, addressing CodeQL alerts, or improving test coverage.
---

# PR Feedback Resolution Skill

Systematic approach to resolving all PR feedback including review comments, CI failures, security findings, and coverage gaps.

## When to Use This Skill

- Address code review comments on pull requests
- Fix failing CI checks (tests, lint, build, formatting)
- Resolve CodeQL security findings
- Improve Codecov patch coverage
- Respond to reviewer questions
- Update PR based on feedback

## Prerequisites

- Active pull request with feedback
- Access to GitHub API for PR details
- Access to CI logs and test results
- Access to CodeQL security scan results
- Access to Codecov reports

## When to Delegate

When performing code changes as part of PR feedback resolution:

- **Delegate to Principal Engineer agent** for all code implementation. The Principal Engineer orchestrates TDD internally.
- **Never invoke** tdd-red, tdd-green, or tdd-refactor agents directly—Principal Engineer handles these.
- **Delegate for research**: Use task-researcher when blocked on understanding requirements.
- **Delegate for planning**: Use task-planner only for complex multi-step remediation.
- For small, unambiguous fixes (docs or metadata) when intent is explicit, apply changes directly.

## Core Workflows

### Workflow 1: Collect All Feedback Sources

Gather complete context before making changes.

1. **Fetch PR review comments**:

```bash
   # Use GitHub MCP or API
   gh pr view <pr-number> --comments
```

2. **Get CI check results**:
   - Check GitHub Actions workflow runs
   - Download test logs from artifacts
   - Review lint/format check outputs
   - Check build logs for errors

3. **Get CodeQL security findings**:
   - Access CodeQL scan results via GitHub UI
   - Review severity and CWE classifications
   - Note file locations and line numbers

4. **Get Codecov patch coverage**:
   - Access Codecov report for PR
   - Identify uncovered lines in diff
   - Note files with low coverage

5. **Document evidence links**:
   - Save URLs to all review threads
   - Save CI run URLs
   - Save CodeQL finding URLs
   - Save Codecov report URL

### Workflow 2: Classify Review Comment Intent

Determine confidence level before making changes.

1. **Analyze comment language**:
   - **Explicit requests**: "Change X to Y", "Add Z", "Remove A"
   - **Questions**: "Should this handle X?" (may be rhetorical)
   - **Suggestions**: "Consider using Y" (may be optional)
   - **Observations**: "This could be simpler" (may not require action)

2. **Compute confidence score**:
   - **90-100%**: Explicit change request with clear instructions
   - **70-89%**: Strong suggestion with clear direction
   - **50-69%**: Suggestion or question with unclear intent
   - **<50%**: Observation or open-ended question

3. **Decision matrix**:
   - **≥70% confidence**: Proceed with change, document rationale
   - **<70% confidence**: Reply asking for clarification, do not change code

### Workflow 3: Address Review Comments

Resolve code review feedback systematically.

1. **For each high-confidence comment (≥70%)**:
   - Validate approach with principal engineer perspective
   - Use TDD cycle: write test → implement → refactor
   - Link commit/change to specific review comment
   - Mark comment as resolved after verification

2. **For each low-confidence comment (<70%)**:
   - Reply in-thread with interpretation and ask for confirmation
   - Example: "I understand this as [interpretation]. Should I [proposed action]?"
   - Wait for clarification before making changes
   - Document decision in PR thread

3. **For suggested code changes**:
   - Review suggestion for correctness and style
   - Accept if it improves code quality
   - If modifying suggestion, explain why in response
   - Apply change and mark resolved

### Workflow 4: Fix CI Test Failures

Systematically resolve failing tests.

1. **Collect test output**:

```bash
   # For ROS 2 workspace
   cat build/<package_name>/test_results/<package_name>/*.xml
```

2. **Diagnose root cause**:
   - Read test failure messages
   - Identify failing assertion or exception
   - Trace back to code change that introduced failure
   - Determine if test expectation or implementation is wrong

3. **Fix implementation or test**:
   - If test expectation is correct: fix implementation
   - If test expectation is wrong: update test
   - If new behavior: update test expectations
   - Add regression test if bug revealed

4. **Verify fix locally**:

```bash
   # ROS 2 example
   source scripts/setup.bash
   colcon test --packages-select <package>
   colcon test-result --verbose
```

5. **Push and verify CI passes**:
   - Commit fix with descriptive message
   - Push to PR branch
   - Monitor CI for green checks

### Workflow 5: Resolve CodeQL Security Findings

Address security vulnerabilities safely.

1. **Analyze finding details**:
   - Review severity (Critical, High, Medium, Low)
   - Understand CWE classification
   - Read CodeQL explanation and remediation guidance
   - Locate vulnerable code in source

2. **Plan remediation**:
   - Prefer minimal-risk fixes
   - Avoid introducing new vulnerabilities
   - Follow OWASP secure coding guidelines
   - Consider defense-in-depth approach

3. **Implement fix**:
   - Apply secure coding pattern
   - Validate inputs and sanitize outputs
   - Add error handling
   - Document security considerations

4. **Add security tests**:
   - Write test that would exploit vulnerability
   - Verify test fails before fix
   - Verify test passes after fix
   - Add additional edge cases

5. **Verify resolution**:
   - Wait for next CodeQL scan
   - Confirm finding is resolved
   - Document fix in PR comment

### Workflow 6: Improve Patch Coverage

Add tests for uncovered code.

1. **Identify coverage gaps**:

```bash
   # From Codecov report or local coverage
   # Note uncovered line ranges in modified files
```

2. **Prioritize coverage**:
   - **Critical paths**: Business logic, error handling
   - **Edge cases**: Boundary conditions, error states
   - **Integration points**: External API calls, file I/O

3. **Write missing tests**:
   - Use TDD Red: Write failing test for uncovered code
   - Use TDD Green: Verify code makes test pass
   - Use TDD Refactor: Improve test clarity

4. **Verify coverage improvement**:

```bash
   # ROS 2 example with coverage
   colcon build --cmake-args -DDRQP_ENABLE_COVERAGE=ON
   colcon test
   # Check coverage report
```

5. **Target coverage goal**:
   - Aim for 80%+ patch coverage minimum
   - 100% for critical business logic
   - Document untestable code with rationale

### Workflow 7: Final Verification

Ensure all feedback is addressed before requesting re-review.

1. **Checklist for completion**:
   - [ ] All review comments addressed or replied
   - [ ] Resolved comments marked as resolved
   - [ ] All CI checks passing (green)
   - [ ] No unresolved CodeQL findings
   - [ ] Patch coverage meets target (≥80%)
   - [ ] All tests passing locally and in CI

2. **Post resolution summary** using the Feedback Resolution Summary template (see Pattern below)

3. **Request re-review**:
   - Tag original reviewers
   - Highlight significant changes from feedback
   - Note any items needing discussion

## Common Patterns

### Pattern: Feedback Resolution Summary Template

Post this template as a PR comment after completing all work:

<!-- validate_skills: ignore-cross-reference-start -->

```markdown
## Feedback Resolution Summary

### Review Comments Addressed

- [Comment #1](link): Changed X to Y per reviewer request
- [Comment #2](link): Added error handling for edge case A
- [Comment #3](link): **Clarification requested** - awaiting response on approach

### CI Failures Fixed

- **Tests**: Fixed 3 failing tests in drqp_serial ([logs](link))
- **Lint**: Resolved formatting issues in 5 files
- **Build**: Updated dependency declarations

### Security Findings Resolved

- **CodeQL-001**: Fixed SQL injection in query builder ([finding](link))
- **CodeQL-002**: Addressed path traversal vulnerability ([finding](link))

### Coverage Improvements

- Added tests for new serial timeout logic (+15 lines covered)
- Added edge case tests for error handling (+8 lines covered)
- **Current patch coverage**: 87% (target: from Codecov report/config)

### Test Results

- All tests passing: [CI run](link)
- Coverage report: [Codecov](link)

### Files Modified

- [drqp_serial/src/driver.cpp](link): Timeout implementation
- [drqp_serial/test/test_driver.cpp](link): Added timeout tests
- [drqp_interfaces/msg/Status.msg](link): Added timeout status field
```

<!-- validate_skills: ignore-cross-reference-end -->

Mark all resolved comments as resolved using GitHub API.

### Pattern: Clarification Template

When comment intent is unclear (<70% confidence):

```markdown
@reviewer Thanks for the feedback! I want to make sure I understand correctly:

**My interpretation**: [Describe your understanding]

**Proposed action**: [What you plan to do]

Could you confirm if this aligns with your intent, or let me know if you had something else in mind?
```

### Pattern: Resolution Comment Template

When marking comment resolved:

```markdown
✅ Addressed in [commit SHA]

**Changes made**: [Brief description]

**Rationale**: [Why this approach was chosen]

**Verification**: [How it was tested]
```

### Pattern: CI Failure Investigation

Systematic debugging of test failures:

1. Read test output completely
2. Identify first failing assertion
3. Review code change that touched that area
4. Reproduce locally if possible
5. Fix root cause, not symptom
6. Add regression test
7. Verify all related tests still pass

### Pattern: Security Finding Response

Safe remediation of CodeQL alerts:

1. Never dismiss without fixing
2. Understand vulnerability class (CWE)
3. Research secure alternatives
4. Implement minimal-risk fix
5. Add tests that would exploit vulnerability
6. Document security considerations
7. Request security review if uncertain

## Execution Log

Maintain an internal execution log documenting:

- **Comment tracking**: Review comment URL → confidence score → action taken → resolution status
- **CI failures**: Check name → failure reason → fix applied → verification result
- **Security findings**: CodeQL ID → severity → remediation → test added
- **Coverage gaps**: File:lines → tests added → coverage delta
- **Evidence trail**: All links to commits, CI runs, comments, findings

## Quality Standards

### Code Changes

- Follow repository engineering standards
- Use TDD cycle for all changes
- Maintain or improve test coverage
- No introduction of new technical debt
- Document non-obvious decisions
- **Safety-first**: 70% intent confidence minimum for code changes; below 70% ask for clarification
- Avoid risky refactors without explicit reviewer request
- Avoid behavior changes unless explicitly requested
- Prefer targeted, minimal diffs

### Communication

- Clear, professional, constructive tone
- Link to evidence (commits, logs, reports)
- Explain rationale for decisions
- Ask for clarification when uncertain
- Thank reviewers for feedback

### Verification

- All tests pass locally before pushing
- CI checks green before requesting re-review
- Coverage targets met
- Security findings resolved
- No regression introduced

## Troubleshooting

### Problem: Cannot determine comment intent

**Solution**: Reply asking for clarification with your interpretation and proposed action. Do not guess or assume.

### Problem: CI fails intermittently (flaky test)

**Solution**: Identify source of non-determinism (timing, randomness, external dependency). Fix test to be deterministic or mark as integration test.

### Problem: CodeQL false positive

**Solution**: Review carefully; often not false positive. If genuinely incorrect, document why and request CodeQL suppression approval.

### Problem: Coverage target cannot be met

**Solution**: Document why certain code is untestable (external API, hardware dependency, etc.). Consider refactoring for testability.

### Problem: Review comment asks for significant refactor

**Solution**: Assess scope vs PR goals. If out of scope, propose follow-up issue. If in scope, create plan and confirm with reviewer before proceeding.

## Success Criteria

- [ ] All review comments resolved or replied
- [ ] All CI checks passing
- [ ] No security findings unresolved
- [ ] Patch coverage ≥80%
- [ ] Changes follow engineering standards
- [ ] PR ready for re-review
- [ ] Evidence documented and linked
- [ ] Timeline met (30 minutes agent time; CI waits excluded)

## Related Resources

- [Code Review Standards](../code-review-standards/) - PR description and review practices
- [Shared Engineering Guidelines](../../instructions/engineering.instructions.md) - Clean Code, SOLID, TDD
- [Principal Engineer](../../agents/principal-engineer.agent.md) - Primary implementation agent with TDD orchestration
