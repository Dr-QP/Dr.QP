---
name: gh-auth
description: 'Authenticate the GitHub CLI (gh) non-interactively in a devcontainer or cloud sandbox by deriving a GH_TOKEN from the VS Code git credential helper. Use when `gh auth status` reports not logged in, before running gh-dependent skills (open-pr, pr-feedback-resolution, extract-github-actions-logs, get-codeql-data, review, refine-issue), or when gh/git commands fail with 401 or "could not authenticate" in a sandbox where `gh auth login` cannot run interactively. Keywords: gh auth, gh auth login, GH_TOKEN, GITHUB_TOKEN, git credential fill, github cli authentication, devcontainer gh login, unauthenticated gh, gh 401.'
---

# Authenticate gh in a devcontainer/sandbox

Use this skill to get `gh` (and any tool that shells out to it) authenticated inside a devcontainer or cloud agent sandbox, without running the interactive `gh auth login` device-code flow, by reusing the GitHub credential VS Code already holds for the host.

See [README.md](./README.md) for how this works under the hood.

## When to Use This Skill

- `gh auth status` reports `You are not logged into any GitHub hosts.`
- A `gh` command fails with `HTTP 401` or `could not authenticate`
- You are about to run a gh-dependent skill (`open-pr`, `pr-feedback-resolution`, `extract-github-actions-logs`, `get-codeql-data`, `review`, `refine-issue`, `generate-pr-description`) in a devcontainer or non-interactive cloud sandbox
- `gh auth login` cannot be used because there is no interactive terminal/browser to complete the OAuth flow

## Prerequisites

- Running inside a VS Code-managed environment (Dev Containers, or a Cursor/Codex cloud session that inherits the VS Code Remote git credential helper) where `git config --get credential.helper` shows a helper that shells out to a `vscode-remote-containers-*.js` (or equivalent Remote-SSH/Codespaces) script
- The user is signed into GitHub on the host VS Code instance (Settings Sync, GitHub Pull Requests extension, or a prior `git push`/`git fetch` over HTTPS)
- `gh` is installed (`which gh`)

## Workflow

1. Check current auth state:

   ```bash
   gh auth status
   ```

   If it reports a logged-in account, stop — no action needed.

2. If not authenticated, ask the host's git credential helper for a GitHub token:

   ```bash
   export GH_TOKEN=$(printf 'protocol=https\nhost=github.com\n\n' | git credential fill 2>/dev/null | grep '^password=' | cut -d= -f2-)
   ```

3. Verify it worked:

   ```bash
   gh auth status
   ```

   Expect `✓ Logged in to github.com account <user> (GH_TOKEN)`.

4. If `GH_TOKEN` came back empty, `git credential fill` had nothing to return — see Troubleshooting below. Stop and report the blocker to the user rather than prompting for a token or attempting `gh auth login` (no interactive flow is available in a sandbox).

## Important Notes

- `GH_TOKEN` is an environment variable scoped to the current shell/process. It does not persist across new terminals or sessions — repeat step 2 in each fresh shell that needs `gh` access.
- Do not write the token to a file, dotfile, shell rc file, or commit it anywhere. Do not echo or print the raw token value in chat output or logs.
- Do not run `gh auth login`, `gh auth login --with-token`, `gh auth setup-git`, or any command that mutates `gh`'s persisted credential store — `GH_TOKEN` as an env var is read-only for the current process and leaves no trace on disk, which is what makes this safe to use in a shared/ephemeral sandbox.
- This follows the same "never store credentials" principle as [agent-skills.instructions.md](../../instructions/agent-skills.instructions.md): the skill relies on the existing host-side credential helper instead of holding its own secret.

## Troubleshooting

| Problem                                            | Likely cause                                                          | Action                                                                                          |
| --------------------------------------------------- | ----------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| `git credential fill` prints nothing                | Host VS Code instance has no cached GitHub credential                   | Ask the user to sign into GitHub in VS Code (Accounts menu) or run `gh auth login` on the host |
| `git credential fill` hangs                          | No credential helper configured (not a VS Code-managed remote)          | Confirm with `git config --get credential.helper`; if empty, this trick does not apply — fall back to asking the user for a token |
| `gh auth status` still shows not logged in after export | `GH_TOKEN` was exported in a different shell/subprocess than the one running `gh` | Re-run the export in the same shell, or `export` it in the parent process before invoking tools |
| `gh` commands return 403 for an operation            | The cached token's OAuth scopes don't cover that operation              | Check scopes with `gh auth status`; ask the user to grant the missing scope via VS Code's GitHub auth, or use a PAT instead |
