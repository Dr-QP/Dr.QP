---
description: 'Delegate PR feedback resolution to Feedback Handler with minimal required context and evidence links'
name: 'pr-feedback'
agent: 'feedback-handler'
argument-hint: 'prNumber branchName reviewUrl ciUrl codeqlUrl codecovUrl'
---

# PR Feedback: Delegate to Feedback Handler

<!-- <mission> -->
You WILL delegate all PR feedback resolution to the Feedback Handler agent with minimal, complete context.
You MUST avoid duplicating guidance already defined in the Feedback Handler agent and PR feedback resolution skill.
<!-- </mission> -->

<!-- <scope-and-preconditions> -->
## Scope & Preconditions

You MUST use this prompt only when the request involves PR review comments, CI failures, CodeQL findings, or coverage gaps.
You MUST NOT use this prompt for purely informational questions.
<!-- </scope-and-preconditions> -->

<!-- <inputs> -->
## Inputs

You MUST collect and pass the following inputs when available:

- PR number: ${input:prNumber:required}
- Branch name: ${input:branchName:required}
- Review comments URL: ${input:reviewUrl:optional}
- CI run URL: ${input:ciUrl:optional}
- CodeQL findings URL: ${input:codeqlUrl:optional}
- Codecov report URL: ${input:codecovUrl:optional}
- Workspace root: ${workspaceFolder}

If any required input is missing, you MUST request it and stop.
<!-- </inputs> -->

<!-- <workflow> -->
## Workflow

1. You MUST invoke the Feedback Handler agent defined in [./.github/agents/feedback-handler.agent.md](../agents/feedback-handler.agent.md).
2. You MUST pass only the minimal context required: PR number, branch name, workspace root, and evidence links.
3. You MUST ask the agent to return a concise summary of actions taken, files modified, and any items requiring user attention.
<!-- </workflow> -->

<!-- <output-expectations> -->
## Output Expectations

You MUST return the Feedback Handler summary to the user verbatim.
You MUST highlight any missing inputs or items requiring user attention.
<!-- </output-expectations> -->

<!-- <quality-assurance> -->
## Quality Assurance

You MUST confirm that:
- The Feedback Handler agent was invoked.
- Required inputs were provided.
- Evidence links were included when available.
<!-- </quality-assurance> -->

<!-- <references> -->
## References

- Feedback Handler agent: [./.github/agents/feedback-handler.agent.md](../agents/feedback-handler.agent.md)
- PR feedback resolution skill: [./.github/skills/pr-feedback-resolution/SKILL.md](../skills/pr-feedback-resolution/SKILL.md)
<!-- </references> -->
