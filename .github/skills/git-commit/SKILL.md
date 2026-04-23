---
name: git-commit
description: Generate conventional commit messages automatically. Use when user runs git commit, stages changes, or asks for commit message help. Analyzes git diff to create clear, descriptive conventional commit messages. Triggers on git commit, staged changes, commit message requests.
---

# Git Commit Skill

Generate [conventional commit messages](./references/conventional-commits.md) from your git diff.

## When to Use This Skill

- ✅ `git commit` without message
- ✅ User asks "what should my commit message be?"
- ✅ Staged changes exist
- ✅ User mentions commit or [conventional commit](./references/conventional-commits.md)
- ✅ Before creating commits

## Prerequisites

- A git repository with staged or unstaged changes to summarize
- Access to the relevant diff, status, or commit context

## What I Generate

### Conventional Commit Format

```text
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**

| Type       | Use when                                           | Typical examples                                  | Reference                               |
| ---------- | -------------------------------------------------- | ------------------------------------------------- | --------------------------------------- |
| `feat`     | Adding new functionality or a new workflow         | auth flow, pagination endpoint, new UI capability | [feat.md](./references/feat.md)         |
| `fix`      | Correcting broken or incorrect behavior            | memory leak, validation issue, connection problem | [fix.md](./references/fix.md)           |
| `docs`     | Changing documentation only                        | README update, API usage examples, guides         | [docs.md](./references/docs.md)         |
| `style`    | Adjusting formatting without logic changes         | CSS polish, formatting cleanup                    | [style.md](./references/style.md)       |
| `refactor` | Restructuring code without changing behavior       | logic extraction, internal reorganization         | [refactor.md](./references/refactor.md) |
| `perf`     | Improving runtime performance                      | query batching, indexing, caching                 | [perf.md](./references/perf.md)         |
| `test`     | Adding or fixing tests                             | unit coverage, test updates, assertions           | [test.md](./references/test.md)         |
| `build`    | Changing build or packaging behavior               | dependency update, build config change            | [build.md](./references/build.md)       |
| `ci`       | Changing CI or CD automation                       | deployment pipeline, workflow update              | [ci.md](./references/ci.md)             |
| `chore`    | Handling maintenance work outside product behavior | repo cleanup, housekeeping, support tasks         | [chore.md](./references/chore.md)       |

## Step-by-Step Workflows

### Workflow: Turn a Diff Into a Conventional Commit

1. Review the staged changes first.
   - Prefer the staged diff because it reflects exactly what will be committed.
   - If nothing is staged yet, review the working tree and either stage a coherent subset or tell the user the message is based on unstaged changes.
   - Identify the primary outcome of the change rather than listing files.

2. Choose the commit `type` from the behavior change.
   - Use `feat` for new user-facing capability or workflow.
   - Use `fix` for corrected behavior or bug resolution.
   - Use `refactor`, `perf`, `test`, `docs`, `build`, `ci`, `style`, or `chore` only when that is the main purpose of the diff.
   - If the diff mixes unrelated purposes, recommend splitting the commit instead of forcing a vague type.

3. Choose an optional `scope` only when it adds real context.
   - Use a stable subsystem, package, feature area, or component name such as `serial`, `brain`, or `docs`.
   - Skip the scope when the change spans multiple areas or no single label improves clarity.
   - Do not invent broad scopes like `misc` or `updates`.

4. Write the subject line as the smallest accurate summary.
   - Use imperative mood and lowercase after the colon, for example `fix(serial): handle reconnect timeout`.
   - Describe the visible outcome or intent, not the implementation detail.
   - Keep it concise and specific; avoid filler like `update stuff` or `fix bug`.

5. Decide whether the commit needs a body.
   - Add a body when the diff is non-trivial, the reason is not obvious from the subject, or there are important constraints, side effects, or follow-up implications.
   - Skip the body for small, self-explanatory changes.
   - In the body, explain what changed and why it was needed; do not restate the diff line-by-line.

6. Decide whether the commit needs a footer.
   - Add footers for issue references, co-authors, or other structured metadata.
   - Prefer issue references such as `Closes #123` or `Refs #123` when the repository uses them.
   - Skip the footer when there is no structured metadata to record.

7. Mark breaking changes explicitly when behavior or interfaces are no longer backward compatible.
   - Use `type(scope)!: subject` or `type!: subject` when the breaking change should be visible in the header.
   - Also add a `BREAKING CHANGE:` footer describing what changed and what callers or operators must do.
   - Treat API removals, incompatible parameter changes, config format changes, and changed operational assumptions as breaking unless proven otherwise.

### Workflow Output Checklist

1. Confirm the subject matches the main outcome of the staged diff.
2. Confirm the chosen type is specific and not a fallback for mixed changes.
3. Confirm the scope is useful or omit it.
4. Add body text only when it improves reviewer understanding.
5. Add footer lines only for structured metadata.
6. Add `!` and `BREAKING CHANGE:` when compatibility is intentionally broken.

## Tips for Best Messages

1. **Be specific**: "fix login button" not "fix bug"
2. **Use imperative mood**: "add" not "added" or "adds"
3. **Include context**: Why this change was needed
4. **Reference issues**: Always include issue numbers
5. **Breaking changes**: Always flag in footer
