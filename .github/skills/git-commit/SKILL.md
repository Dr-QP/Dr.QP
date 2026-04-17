---
name: git-commit
description: Generate conventional commit messages automatically. Use when user runs git commit, stages changes, or asks for commit message help. Analyzes git diff to create clear, descriptive conventional commit messages. Triggers on git commit, staged changes, commit message requests.
allowed-tools: Bash, Read
---

# Git Commit Skill

Generate [conventional commit messages](./References/conventional-commits.md) from your git diff.

## When I Activate

- ✅ `git commit` without message
- ✅ User asks "what should my commit message be?"
- ✅ Staged changes exist
- ✅ User mentions commit or [conventional commit](./References/conventional-commits.md)
- ✅ Before creating commits

## What I Generate

### Conventional Commit Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style (formatting, no logic change)
- `refactor`: Code refactoring
- `perf`: Performance improvements
- `test`: Test additions or fixes
- `build`: Build system changes
- `ci`: CI/CD changes
- `chore`: Maintenance tasks

Detailed references by commit type:

- `feat`: [feat.md](./References/feat.md)
- `fix`: [fix.md](./References/fix.md)
- `docs`: [docs.md](./References/docs.md)
- `style`: [style.md](./References/style.md)
- `refactor`: [refactor.md](./References/refactor.md)
- `perf`: [perf.md](./References/perf.md)
- `test`: [test.md](./References/test.md)
- `build`: [build.md](./References/build.md)
- `ci`: [ci.md](./References/ci.md)
- `chore`: [chore.md](./References/chore.md)

## Tips for Best Messages

1. **Be specific**: "fix login button" not "fix bug"
2. **Use imperative mood**: "add" not "added" or "adds"
3. **Include context**: Why this change was needed
4. **Reference issues**: Always include issue numbers
5. **Breaking changes**: Always flag in footer
