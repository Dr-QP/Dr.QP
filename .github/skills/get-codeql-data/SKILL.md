---
name: get-codeql-data
description: 'Fetch CodeQL code scanning data with the GitHub CLI. Use when asked to get CodeQL alerts, inspect code scanning findings for a pull request, list open security findings, or retrieve CodeQL alert metadata from GitHub. Keywords: CodeQL, code scanning, security alerts, gh api, pull request alerts, codeql data.'
---

# Get CodeQL Data

Use this skill to retrieve CodeQL code scanning alerts and related metadata with `gh api`.

## When to Use This Skill

- List open CodeQL alerts for a pull request
- Inspect security findings for a branch, commit, or repository
- Collect alert metadata for triage, review, or remediation
- Extract file paths, line numbers, severities, and alert URLs

## Prerequisites

- `gh` must be installed
- `gh` authentication is required
- The authenticated account must have access to the repository

Always verify authentication first:

```bash
gh auth status
```

If authentication fails, stop and tell the user to run:

```bash
gh auth login
```

## Workflow

1. Confirm the repository owner, repository name, and the scope the user wants:
   - pull request number
   - branch or ref
   - specific alert number
   - all open alerts in the repository
2. Verify `gh auth status` succeeds.
3. Query the code scanning REST API with `gh api`.
4. Filter the response to the fields the user actually needs.
5. Return a compact summary with the alert number, rule, severity, location, and URL.

## Common Commands

List open CodeQL alerts for a pull request:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts \
  -f pr=<pr-number> \
  -f state=open \
  -f tool_name=CodeQL
```

Project the most useful triage fields with `--jq`:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts \
  -f pr=<pr-number> \
  -f state=open \
  -f tool_name=CodeQL \
  --jq '.[] | {number: .number, rule: .rule.id, severity: .rule.security_severity_level, path: .most_recent_instance.location.path, start_line: .most_recent_instance.location.start_line, url: .html_url}'
```

Example for this repository:

```bash
gh api repos/Dr-QP/Dr.QP/code-scanning/alerts \
  -f pr=368 \
  -f state=open \
  -f tool_name=CodeQL \
  --jq '.[] | {number: .number, rule: .rule.id, severity: .rule.security_severity_level, path: .most_recent_instance.location.path, start_line: .most_recent_instance.location.start_line, url: .html_url}'
```

List open CodeQL alerts for the whole repository:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts \
  -f state=open \
  -f tool_name=CodeQL
```

List alerts for a branch or ref:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts \
  -f ref=refs/heads/<branch-name> \
  -f state=open \
  -f tool_name=CodeQL
```

Fetch one alert by number:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts/<alert-number>
```

Show more detailed remediation-oriented fields:

```bash
gh api repos/<owner>/<repo>/code-scanning/alerts \
  -f pr=<pr-number> \
  -f state=open \
  -f tool_name=CodeQL \
  --jq '.[] | {number: .number, rule: .rule.id, rule_description: .rule.description, severity: .rule.security_severity_level, tags: .rule.tags, state: .state, path: .most_recent_instance.location.path, start_line: .most_recent_instance.location.start_line, message: .most_recent_instance.message.text, url: .html_url}'
```

## Triage Guidance

- Prefer `-f pr=<pr-number>` when the user is fixing PR feedback or reviewing a single pull request.
- Add `-f state=open` unless the user explicitly needs closed or dismissed alerts.
- Add `-f tool_name=CodeQL` to avoid mixing in alerts from other scanners.
- Use `--jq` to keep the output short and targeted.
- If the user asks for all raw data, omit `--jq` and summarize the important fields afterward.

## Troubleshooting

| Problem | Likely cause | Action |
| --- | --- | --- |
| `gh api` returns 401 or 403 | Missing auth or insufficient permissions | Run `gh auth status`, then authenticate or use an account with repo access |
| No alerts returned for a PR | No open CodeQL alerts on that PR, wrong PR number, or alerts already fixed | Verify the PR number and retry without `-f state=open` if history is needed |
| Mixed scanner results | The repo uses multiple code scanning tools | Add `-f tool_name=CodeQL` |
| Branch query returns nothing | Incorrect ref format | Use `refs/heads/<branch-name>` for branches |

## References

- GitHub REST API: `repos/{owner}/{repo}/code-scanning/alerts`
- GitHub CLI: `gh api`
