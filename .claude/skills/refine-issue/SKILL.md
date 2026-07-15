---
name: refine-issue
description: Refine an existing GitHub issue into a concise, evidence-backed implementation specification with testable acceptance criteria and relevant constraints. Use when asked to improve, enrich, or clarify an issue; do not use to create an issue, implement its work, or write a local design specification.
---

# Refine Issue

Turn an existing GitHub issue into a compact specification that an implementing
agent can act on. Preserve useful context, make requirements testable, and add
only claims supported by the issue, its linked context, or repository evidence.

## Establish Access and Scope

Accept an issue URL, an `OWNER/REPO#NUMBER` reference, or a bare issue number.
For a bare number, use the current repository; state the resolved repository in
the draft or update summary so the target is unambiguous.

Prefer the GitHub CLI when it is installed and authenticated:

```bash
gh auth status
gh repo view --json nameWithOwner --jq .nameWithOwner
gh issue view 123 --repo OWNER/REPO --comments \
  --json number,title,body,url,state,labels,comments
```

For an issue URL, pass the URL directly to `gh issue view`. For an
`OWNER/REPO#NUMBER` reference, split it into `--repo OWNER/REPO` and the issue
number. Do not guess a repository for a bare number: resolve it with
`gh repo view` first.

If `gh` is unavailable or unauthenticated, use a connected GitHub capability
only when the active environment documents an issue read/update operation. Do
not assume particular MCP tool names. If neither path can read the issue,
explain the limitation and request the issue text or an authenticated GitHub
environment. If reading works but updating does not, return a ready-to-paste
Markdown draft instead of claiming the issue was updated.

This skill refines existing issues. Creating an issue belongs to an issue
creation workflow; changing source code belongs to implementation work; and a
repository-local proposal or design document belongs to local specification
refinement.

## Gather Evidence Before Drafting

1. Read the issue title, body, labels, and comments. Retain linked decisions,
   reproduction steps, and constraints that still apply.
2. Inspect explicitly linked issues, pull requests, documentation, and files.
   Search the repository for concrete terms from the issue before making a
   technical claim. For example:

   ```bash
   rg -n -i 'exact term from the issue' .
   ```

3. Separate evidence from inference. State repository facts only when observed;
   phrase supported implications narrowly; omit unknown architecture, API,
   performance, security, or reliability requirements.
4. Ask one focused clarifying question before an update when a material scope
   choice, acceptance criterion, or technical direction has more than one
   reasonable interpretation. Ordinary, explicit reasoning is sufficient; no
   specialized reasoning tool is required.

## Write the Refinement

Rewrite the body in natural language for an implementing agent. Keep the
existing title unless the user asks to change it. Use only the sections that
add meaningful, evidence-backed information:

```markdown
## Goal

Explain the intended outcome and its reason in one or two sentences.

## Acceptance Criteria

- [ ] Describe one independently verifiable outcome.
- [ ] Describe another independently verifiable outcome.

## Technical Considerations

- Record an observed dependency, constraint, or integration detail.

## Edge Cases

- Describe a relevant boundary condition or failure mode.

## Non-functional Requirements

- State a measurable requirement only when the evidence establishes one.
```

Carry forward relevant links and decisions. Remove redundant prose and prior
editing commentary. Do not repeat the title, invent implementation details,
add hidden markup, or pad the issue with generic checklists. Keep the body
under 400 words unless the user explicitly requests more.

## Deliver or Update

Show the proposed body when the user asks for a draft or when clarification is
needed. Update GitHub only when the user asks to update the issue and the
material requirements are clear. With `gh`, write the body through a repository
temporary file and then update the resolved issue:

```bash
mkdir -p ./.tmp
gh issue edit 123 --repo OWNER/REPO --body-file ./.tmp/refined-issue.md
```

Report the resolved `OWNER/REPO#NUMBER`, whether the issue was updated or only
drafted, and any unanswered question. Do not claim an update succeeded unless
the write command or connected capability confirms it.
