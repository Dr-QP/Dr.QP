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

## Tips for Best Messages

1. **Be specific**: "fix login button" not "fix bug"
2. **Use imperative mood**: "add" not "added" or "adds"
3. **Include context**: Why this change was needed
4. **Reference issues**: Always include issue numbers
5. **Breaking changes**: Always flag in footer
