# How `gh-auth` works

This skill bootstraps `gh` CLI authentication in environments where the interactive
`gh auth login` device-code/browser flow is not available — devcontainers, and
cloud/remote agent sandboxes (Cursor Cloud, Codex cloud sessions, etc.) that have no
TTY or browser to complete an OAuth handshake.

## The trick

```bash
export GH_TOKEN=$(printf 'protocol=https\nhost=github.com\n\n' | git credential fill 2>/dev/null | grep '^password=' | cut -d= -f2-)
```

This asks `git` to resolve credentials for `https://github.com`, and `gh` honors a
`GH_TOKEN` (or `GITHUB_TOKEN`) environment variable as an auth source with higher
priority than its own on-disk config — so exporting it is enough; no `gh auth login`
or `gh auth setup-git` call is needed.

## Why `git credential fill` returns something useful here

`git credential fill` doesn't generate a credential — it queries whatever credential
helpers are configured in `git config` and returns the first match. In a VS Code-managed
remote (Dev Containers, Remote-SSH, Codespaces, or a cloud agent session that inherits
the same setup), VS Code injects a credential helper into the container's git config
that looks roughly like this:

```text
credential.helper=!f() { /root/.vscode-server/bin/<commit>/node /tmp/vscode-remote-containers-<id>.js git-credential-helper $*; }; f
```

That helper is a small Node script which doesn't store anything locally. Instead it
forwards the credential request over VS Code's remote IPC channel back to the **host**
VS Code process. If the user is signed into GitHub on the host — via Settings Sync,
the GitHub Pull Requests extension, the built-in Accounts menu, or simply because VS
Code has previously done a `git push`/`git fetch` over HTTPS and cached the result —
the host resolves the request immediately and ships `username=`/`password=` back down
into the container. The `password` value is the actual GitHub OAuth token VS Code holds
for that account, not a literal password.

Because this round-trip happens entirely over the existing Remote-* IPC channel (not a
new OAuth flow), it's non-interactive: it either returns a token immediately or returns
nothing within `git credential fill`'s normal timeout. There's no prompt, no browser
window, no device code to relay — which is exactly what makes it usable from a
non-interactive agent shell where `gh auth login` would otherwise hang waiting for
input that can never arrive.

## Why this is safe to rely on

- **No new secret is created or stored.** The skill only ever reads a token that
  already exists in the host's VS Code credential cache; it doesn't mint, persist, or
  write that token to disk inside the container.
- **Scoped to the process environment.** `export GH_TOKEN=...` only affects the current
  shell and its children. Closing the shell drops it; it's never written to `~/.bashrc`,
  `~/.gitconfig`, or any file `gh` itself would persist.
- **No `gh` config mutation.** Unlike `gh auth login --with-token`, which writes the
  token into `~/.config/gh/hosts.yml`, using `GH_TOKEN` as an env var leaves `gh`'s own
  credential store untouched. The next shell starts unauthenticated again unless it
  repeats the same export.
- **Fails closed.** If the host has no cached GitHub credential (user never signed in,
  or the remote isn't VS Code-managed at all), `git credential fill` simply returns
  nothing and `GH_TOKEN` ends up empty — `gh auth status` then correctly reports "not
  logged in" instead of silently succeeding with a bad token.

## Scope of the resulting token

The token's OAuth scopes are whatever the host's GitHub VS Code session was granted —
typically `repo`, `workflow`, `read:org`, `gist`, etc. for a normal developer sign-in.
If a `gh` operation needs a scope the cached token doesn't have (e.g. an org-admin
action), this trick won't grant it; the user needs to re-authorize the VS Code GitHub
session with the additional scope.
