---
description: 'Guidance for remote agents handling PR review comments, CI failures, security findings, and coverage gaps'
applyTo: '**/*'
---

# PR Feedback Resolution Instructions

Instructions for remote agents addressing pull request feedback, CI failures, security findings, and test coverage gaps.

## When These Instructions Apply

These instructions apply when you are operating as a remote agent (cloud agent) and the task involves:

1. **Addressing PR review comments** - Responding to or resolving code review feedback
2. **Fixing CI failures** - Resolving failing tests, lint errors, build failures, or formatting issues
3. **Resolving security findings** - Addressing CodeQL alerts or other security vulnerabilities
4. **Improving test coverage** - Adding tests to meet coverage requirements or close coverage gaps
5. **General PR feedback resolution** - Any task that involves processing feedback on a pull request

## Primary Directive: Use the Feedback Handler Agent

**When addressing PR feedback, you MUST delegate to the Feedback Handler agent.**

### Why Use the Feedback Handler Agent

The Feedback Handler agent is specifically designed to:
- **Systematically address all feedback types** (review comments, CI failures, security, coverage)
- **Manage intent classification** to avoid misinterpreting reviewer comments
- **Orchestrate TDD methodology** through the principal-engineer subagent
- **Provide safety controls** to prevent risky changes without explicit approval
- **Track and document resolution** with evidence links and comprehensive reporting
- **Meet quality standards** as defined in repository engineering guidelines

### When NOT to Use the Feedback Handler Agent

Do not use the Feedback Handler agent if:
- You are NOT a remote/cloud agent (use it only when operating autonomously)
- The task is purely informational (e.g., "explain what this PR comment means")
- The user explicitly requests a different approach or agent
- You are already operating AS the feedback-handler agent (avoid recursive delegation)

## How to Invoke the Feedback Handler Agent

When you determine that PR feedback resolution is required, follow this pattern:

### Step 1: Gather Context

Before delegating, collect:
- PR number and branch name
- Links to review comments, CI runs, CodeQL findings, or Codecov reports
- Current workspace path
- Any specific instructions from the user

### Step 2: Delegate to Feedback Handler

Use the agent invocation tool with this prompt pattern:

```
This task must be performed as the agent "Feedback Handler" defined in ".github/agents/feedback-handler.agent.md".

IMPORTANT:
- Read and apply the entire .agent.md spec (tools, constraints, quality standards)
- Context: PR #${prNumber}, branch ${branchName}
- Base path: ${workspaceRoot}
- Task: [Address PR review comments / Fix CI failures / Resolve security findings / Improve coverage]
- Evidence links:
  - [Review comments URL]
  - [CI run URL]
  - [CodeQL findings URL]
  - [Codecov report URL]

Your mission:
1. Collect all feedback from review comments, CI failures, security scans, and coverage reports
2. Classify intent and confidence for each review comment
3. Address all feedback following repository engineering standards
4. Verify fixes with targeted tests
5. Post comprehensive resolution summary
6. Mark resolved comments as resolved

Return: Concise summary of actions taken, files modified, and any items requiring user attention.
```

### Step 3: Monitor and Verify

After the Feedback Handler agent completes:
- Review the resolution summary
- Verify that all feedback has been addressed
- Confirm CI checks are passing
- Report completion to the user

## Example Scenarios

### Scenario 1: User Reports "Address PR Feedback"

```
User: "Address the review comments on PR #123"

Your response (as remote agent):
1. Fetch PR #123 details
2. Identify review comments, CI status, security findings, coverage gaps
3. Delegate to feedback-handler agent with context
4. Monitor progress and report completion
```

### Scenario 2: User Reports "Fix Failing CI"

```
User: "The CI is failing on my PR, please fix it"

Your response (as remote agent):
1. Identify PR number and failing CI checks
2. Gather CI logs and failure details
3. Delegate to feedback-handler agent with CI context
4. Monitor progress and report completion
```

### Scenario 3: User Reports "Improve Test Coverage"

```
User: "Add tests to meet coverage requirements for PR #456"

Your response (as remote agent):
1. Access Codecov report for PR #456
2. Identify uncovered lines in diff
3. Delegate to feedback-handler agent with coverage context
4. Monitor progress and report completion
```

## Quality Expectations

When working with the Feedback Handler agent, ensure:

### Communication
- **Clear delegation**: Provide complete context in the delegation prompt
- **Evidence links**: Include URLs to all relevant feedback sources
- **Progress updates**: Report status to user at key milestones

### Verification
- **All feedback addressed**: Every review comment, CI failure, security finding, and coverage gap
- **Tests passing**: All CI checks green before completion
- **Standards compliance**: All changes follow repository engineering guidelines
- **Documentation**: Resolution summary posted with evidence links

### Safety
- **Intent validation**: Comments with <80% confidence trigger clarification requests
- **Minimal changes**: Target specific issues without unnecessary refactoring
- **No guessing**: Ask for clarification rather than assuming intent
- **Risk assessment**: Flag high-risk changes for user review

## Related Resources

- [Feedback Handler Agent](../.github/agents/feedback-handler.agent.md) - Full agent specification
- [PR Feedback Resolution Skill](../.github/skills/pr-feedback-resolution/SKILL.md) - Detailed workflows
- [Code Review Standards Skill](../.github/skills/code-review-standards/SKILL.md) - PR practices and standards
- [Engineering Guidelines](./engineering.instructions.md) - Clean Code, SOLID, TDD principles
- [Principal Engineer Agent](../.github/agents/principal-engineer.agent.md) - Primary implementation subagent

## Summary

**Key Takeaway**: When operating as a remote agent and the task involves PR feedback resolution, ALWAYS delegate to the Feedback Handler agent. This ensures systematic, safe, and standards-compliant resolution of all feedback types.
