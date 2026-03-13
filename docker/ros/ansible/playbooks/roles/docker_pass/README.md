# Docker Pass CLI Plugin Role

This Ansible role builds and installs the `docker-pass` secrets provider plugin from [docker/secrets-engine](https://github.com/docker/secrets-engine). It enables `docker mcp gateway run --secrets docker-desktop` to resolve secrets inside a Linux devcontainer.

## Overview

On macOS, Docker Desktop stores MCP secrets in the host OS Keychain. Inside a Linux devcontainer, that Keychain is not accessible. The `docker-pass` plugin bridges this gap by providing a `docker pass` CLI subcommand backed by the Linux [`pass`](https://www.passwordstore.org/) password manager.

This role installs `docker-pass` automatically. However, **secrets must be initialised manually after the devcontainer is built** — they are not automatically synchronised from Docker Desktop.

## Requirements

- Go (installed automatically by this role)
- `gpg` and `pass` (installed automatically by this role)

## Role Variables

| Variable                | Default                                        | Description                            |
| ----------------------- | ---------------------------------------------- | -------------------------------------- |
| `docker_pass_repo_url`  | `https://github.com/docker/secrets-engine.git` | Git repository URL                     |
| `docker_pass_version`   | `main`                                         | Git branch, tag, or commit to checkout |
| `docker_pass_user`      | `{{ ros_user }}`                               | User to install the plugin for         |
| `docker_pass_user_home` | `{{ user_home }}`                              | User's home directory                  |

## Example Usage

```yaml
- name: Install docker-pass CLI plugin
  hosts: all
  become: true
  roles:
    - { role: docker_pass, tags: ['docker_pass'] }
```

## What This Role Does

1. **Installs Go** - Required for building the plugin
2. **Installs `gpg` and `pass`** - Runtime dependencies for the secrets store
3. **Creates plugins directory** - `~/.docker/cli-plugins/`
4. **Clones the repository** - Shallow clone of secrets-engine
5. **Builds the plugin** - `go build` from `plugins/pass/`
6. **Verifies installation** - Confirms plugin exists
7. **Cleans up** - Removes temporary build directory

## Post-Install: Initialising Secrets

`docker-pass` uses the local `pass` store (GPG-encrypted). The store must be initialised once per devcontainer, and each secret must be added manually.

### 1 — Generate or import a GPG key

```bash
# Generate a new key (accept defaults; use a blank passphrase for unattended use)
gpg --batch --gen-key <<EOF
Key-Type: default
Subkey-Type: default
Name-Real: Docker Secrets
Name-Email: docker-secrets@local
Expire-Date: 0
%no-protection
EOF
```

### 2 — Initialise the pass store

```bash
GPG_ID=$(gpg --list-keys --with-colons docker-secrets@local | awk -F: '/^fpr/ { print $10; exit }')
pass init "$GPG_ID"
```

### 3 — Add your secrets

Secrets are stored under the `docker/mcp/` prefix to match Docker Desktop's naming:

```bash
echo "ghp_your_token_here" | pass insert -e docker/mcp/github.personal_access_token
```

Verify with:

```bash
docker pass ls
# or
docker mcp secret ls
```

### 4 — Use with docker mcp gateway

No extra flags needed — `--secrets docker-desktop` is the default:

```bash
docker mcp gateway run
```

## References

- [docker/secrets-engine GitHub Repository](https://github.com/docker/secrets-engine)
- [pass — the standard unix password manager](https://www.passwordstore.org/)
