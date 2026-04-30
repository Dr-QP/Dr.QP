---
name: refine-issue
description: Refine a GitHub issue by enriching it with acceptance criteria, technical considerations, edge cases, and NFRs. Keep the result compact and agent-readable. Use when asked to refine, enrich, or improve an issue description. Keywords: refine issue, improve issue, enrich issue, acceptance criteria, issue refinement.
---

# Refine Issue

Enrich a GitHub issue with structured details that give an implementing agent enough context to act. Keep the refined body concise — every sentence must add information an agent needs. Omit boilerplate, marketing language, and human-oriented explanations.

## When to Use This Skill

- Refine or improve an existing GitHub issue
- Add acceptance criteria, technical considerations, or edge cases to a sparse issue
- Prepare an issue so a coding agent can implement it without follow-up questions

## Prerequisites

- Issue URL or number in the conversation
- GitHub MCP tools available: `github/issue_read`, `github/issue_write`

## Workflow

### Step 1: Read the Issue

Use `github/issue_read` to fetch the issue body, title, and any existing comments.

### Step 2: Plan the Refinement

Use the `sequentialthinking` tool to reason through what the refined issue needs:

- What is the goal? What problem does it solve?
- What are the minimal, testable acceptance criteria?
- What technical constraints or dependencies apply?
- What edge cases or risks must be handled?
- What NFRs (performance, security, reliability) are relevant?

Keep each criterion specific and verifiable. Prune anything that is obvious or already implied by the codebase conventions.

### Step 3: Draft the Refined Body

Rewrite the issue body using the structure below. Omit any section that has nothing meaningful to say.

```markdown
## Goal

One or two sentences: what this issue achieves and why it matters.

## Acceptance Criteria

- [ ] <Specific, testable condition>
- [ ] <Specific, testable condition>

## Technical Considerations

- <Constraint, dependency, or implementation note an agent must know>

## Edge Cases

- <Boundary condition or failure mode to handle>

## NFRs

- <Non-functional requirement if applicable>
```

Rules for the body:

- Use imperative, precise language targeted at an implementing agent.
- Each acceptance criterion must be independently verifiable.
- Do not repeat the issue title or restate what the code already enforces by convention.
- Do not add HTML comments, hidden markup, or extraneous prose — plain Markdown only.
- Keep the total body under 400 words unless the issue is genuinely complex.

### Step 4: Update the Issue

Call `github/issue_write` with the refined body. Do not ask for user confirmation — apply the update directly.

## Output

Updated issue body on GitHub. No user-facing summary is needed unless the caller explicitly requests one.
