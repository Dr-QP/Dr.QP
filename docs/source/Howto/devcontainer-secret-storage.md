# Devcontainer secret storage and the keyring tradeoff

This page explains how secrets are stored inside the devcontainer, what the
default storage does and does not protect against, and how to harden it if your
threat model demands it.

## What is stored, and where

The devcontainer keeps two kinds of secrets:

- **docker/mcp secrets** — always served from the host's `docker-secrets-engine`
  socket, with no in-container fallback or import step. `.devcontainer/devcontainer-init.sh`
  detects the socket and mounts it directly into both the `devcontainer` and
  `mcp-gateway` services as `DOCKER_SECRETS_ENGINE_SOCKET`; if the socket isn't
  present, a non-functional stub is mounted instead and docker/mcp secrets are
  simply unavailable until the host engine is running. Either way, these
  secrets never persist inside the container.
- **`gh auth login` tokens** — GitHub CLI credentials, stored in the GNOME
  Keyring inside the container.

The in-container keyring file is:

```
~/.local/share/keyrings/login.keyring
```

It is brought up by `scripts/devcontainer-setup-keyring.sh`, which exposes the
`org.freedesktop.secrets` Secret Service over a D-Bus session.

## The default tradeoff: an unencrypted keyring

`devcontainer-setup-keyring.sh` creates `login.keyring` with an **empty
password**. A GNOME Keyring with an empty password is stored **unencrypted** on
disk — the secrets are effectively plaintext.

This is deliberate. An empty password lets the keyring daemon start _headless_,
without an interactive unlock prompt on every container start or window reload.

### What this protects against

- Nothing at the file-encryption layer inside the container.

### What actually protects the secrets

- **Host-level disk encryption** — on a typical single-user dev machine the
  container's filesystem lives on a host-encrypted disk (macOS FileVault, or the
  Docker Desktop VM disk). At rest, the keyring is protected by that layer.
- **Container/host isolation** — the file is only reachable by processes with
  access to the container's filesystem.

For a single-user devcontainer this is an acceptable tradeoff: any in-container
encryption password would have to live nearby anyway, so encrypting the keyring
file in place adds little real protection while adding startup complexity.

## Options for hardening the `gh` token keyring (not currently enabled)

These only apply to the in-container keyring above; docker/mcp secrets already
never persist in the container (see [What is stored, and where](#what-is-stored-and-where)).

### Password-encrypted keyring

If at-rest encryption inside the container becomes a requirement, switch from the
hand-written empty-password keyring to `gnome-keyring-daemon --login`, which
reads an unlock password on stdin and lets the daemon create/unlock an
AES-encrypted `login.keyring`.

The encryption is only as strong as wherever the unlock password lives. The
recommended source is the **host keychain / 1Password**, fetched at init time and
never written to a repo-local file.

### Ephemeral tmpfs keyring

Place the keyring on tmpfs so nothing touches disk, and re-import secrets each
session. This removes the at-rest file entirely at the cost of a per-session
re-import.

## Summary

| Option                           | Secrets at rest in container | Headless start                                | Notes                          |
| -------------------------------- | ---------------------------- | --------------------------------------------- | ------------------------------ |
| Empty-password keyring (default) | Yes, unencrypted             | Yes                                           | Relies on host disk encryption |
| Password-encrypted keyring       | Yes, encrypted               | Yes, if password is sourced non-interactively | Use host keychain / 1Password  |
| Ephemeral tmpfs                  | No                           | Yes                                           | Re-import every session        |

docker/mcp secrets aren't in this table: they're always served from the host's
`docker-secrets-engine` socket and never persist in the container, regardless
of which keyring option above is active.
