# Devcontainer secret storage and the keyring tradeoff

This page explains how secrets are stored inside the devcontainer, what the
default storage does and does not protect against, and how to harden it if your
threat model demands it.

## What is stored, and where

The devcontainer keeps two kinds of secrets:

- **docker/mcp secrets** — used by the `docker-pass` secrets engine. Depending on
  configuration, these live either in a GNOME Keyring inside the container or in
  an externally mounted secrets engine on the host (see below).
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

This is deliberate. An empty password lets the keyring daemon start *headless*,
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

## Options

### 1. External mounted secrets engine (preferred, already implemented)

When the host runs the `docker-secrets-engine` and mounts its socket,
`.devcontainer/devcontainer-init.sh` detects it and sets
`DOCKER_PASS_EXTERNAL_ENGINE=1`. In that mode the host engine is the source of
truth and **docker/mcp secrets never persist inside the container** — the
strongest real-world option.

To use it, run the `docker-secrets-engine` on the host so its socket exists at
the expected path (e.g. `~/Library/Caches/docker-secrets-engine/engine.sock` on
macOS). `devcontainer-init.sh` flips `DOCKER_PASS_EXTERNAL_ENGINE=1`
automatically when the socket is present; otherwise it falls back to the
in-container keyring and an encrypted one-shot secret import.

Note: this covers docker/mcp secrets only. `gh` tokens still land in the
in-container keyring.

### 2. Password-encrypted keyring (hardening path, not currently enabled)

If at-rest encryption inside the container becomes a requirement, switch from the
hand-written empty-password keyring to `gnome-keyring-daemon --login`, which
reads an unlock password on stdin and lets the daemon create/unlock an
AES-encrypted `login.keyring`.

The encryption is only as strong as wherever the unlock password lives. The
recommended source is the **host keychain / 1Password**, fetched at init time and
never written to a repo-local file.

> **Do not** reuse `DOCKER_PASS_EXPORT_KEY` as the unlock password. It is
> regenerated randomly on every `devcontainer-init`, so it cannot unlock a
> keyring that persists across reloads.

### 3. Ephemeral tmpfs keyring (not currently enabled)

Place the keyring on tmpfs so nothing touches disk, and re-import secrets each
session. This removes the at-rest file entirely at the cost of a per-session
re-import.

## Summary

| Option | Secrets at rest in container | Headless start | Notes |
| ------ | ---------------------------- | -------------- | ----- |
| Empty-password keyring (default) | Yes, unencrypted | Yes | Relies on host disk encryption |
| External mounted engine | No (docker secrets) | Yes | Preferred; `gh` tokens still local |
| Password-encrypted keyring | Yes, encrypted | Yes, if password is sourced non-interactively | Use host keychain / 1Password |
| Ephemeral tmpfs | No | Yes | Re-import every session |
