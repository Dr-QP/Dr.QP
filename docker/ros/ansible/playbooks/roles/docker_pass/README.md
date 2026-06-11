# Docker Pass CLI Plugin Role

This Ansible role installs a working Linux `docker-pass` Docker CLI plugin using the upstream [docker/secrets-engine](https://github.com/docker/secrets-engine) pass module plus a local launcher. It fixes the fact that `plugins/pass/` in `v0.4.0` is a library package, so running `go build` in that directory produces a Go archive rather than a Docker CLI executable.

## Overview

The upstream Docker Pass implementation on Linux uses the freedesktop Secret Service API over D-Bus. It is not backed by the `pass` GPG password store. This role installs the Docker CLI plugin binary, and the launcher also exposes a hidden local secrets-engine mode that serves the resolver API expected by `docker mcp secret ...` inside Linux devcontainers.

This role installs `docker-pass` automatically. The devcontainer startup scripts are responsible for launching a D-Bus session plus `gnome-keyring-daemon` so the official Linux backend is available, then starting the local resolver socket on `~/.cache/docker-secrets-engine/engine.sock`.

## Requirements

- Go (installed automatically by this role)
- A Linux Secret Service backend such as `gnome-keyring` or `kdewallet` if you want the official upstream `docker pass` runtime to work

## Role Variables

- `docker_pass_repo_url` - Git repository URL
- `docker_pass_version` - Git branch, tag, or commit to checkout
- `docker_pass_user` - User to install the plugin for
- `docker_pass_user_home` - User's home directory

See `defaults/main.yml` for default values

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
2. **Installs Secret Service runtime packages** - `dbus`, `dbus-x11`, `gnome-keyring`, and `libglib2.0-bin`
3. **Creates plugins directory** - `~/.docker/cli-plugins/`
4. **Clones the repository** - Shallow clone of secrets-engine
5. **Renders a Linux launcher** - Adds a local `main.go` that exposes Docker CLI metadata, executes the upstream pass commands, and can serve the local resolver socket used by Docker MCP
6. **Builds the plugin** - `go build ./cmd/docker-pass`
7. **Verifies installation** - Runs `docker-cli-plugin-metadata` against the built binary
8. **Cleans up** - Removes temporary build directory

## Runtime Notes

`docker pass` on Linux talks to Secret Service, not the `pass` utility. If `docker pass ls` fails with D-Bus or collection errors, the plugin installation is correct but the container is missing a running Secret Service session.

The repository bootstrap imports exported secrets with `docker pass set` only when it is operating without a mounted external secrets engine socket, so the devcontainer still has a local backend when it cannot talk to the host engine directly.

The devcontainer startup script also launches the hidden `internal-engine-serve` mode so `docker mcp secret ls` and other local resolver clients can talk to `~/.cache/docker-secrets-engine/engine.sock` without Docker Desktop running inside the container. When a host engine socket is already mounted and healthy, startup uses that socket directly and skips local secret mirroring.

## References

- [docker/secrets-engine GitHub Repository](https://github.com/docker/secrets-engine)
- [pass — the standard unix password manager](https://www.passwordstore.org/)
