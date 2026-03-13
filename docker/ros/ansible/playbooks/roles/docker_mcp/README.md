# Docker MCP CLI Plugin Role

This Ansible role builds and installs the Docker MCP (Model Context Protocol) Gateway as a Docker CLI plugin.

## Overview

The Docker MCP Gateway enables integration with AI tools and agents by providing a standardized protocol for model context. This role clones the [docker/mcp-gateway](https://github.com/docker/mcp-gateway) repository, builds the `docker-mcp` CLI plugin, and installs it to the user's Docker CLI plugins directory.

## Requirements

- Docker Desktop 4.59+ (with MCP Toolkit feature enabled)
- Go (installed automatically by this role)
- Git and Make (installed automatically by this role)

## Role Variables

| Variable               | Default                                     | Description                            |
| ---------------------- | ------------------------------------------- | -------------------------------------- |
| `docker_mcp_repo_url`  | `https://github.com/docker/mcp-gateway.git` | Git repository URL                     |
| `docker_mcp_version`   | `main`                                      | Git branch, tag, or commit to checkout |
| `docker_mcp_user`      | `{{ ros_user }}`                            | User to install the plugin for         |
| `docker_mcp_user_home` | `{{ user_home }}`                           | User's home directory                  |

## Example Usage

```yaml
- name: Install Docker MCP CLI plugin
  hosts: all
  become: true
  roles:
    - { role: docker_mcp, tags: ['docker_mcp'] }
```

### Install specific version

```yaml
- name: Install Docker MCP CLI plugin (specific version)
  hosts: all
  become: true
  roles:
    - role: docker_mcp
      tags: ['docker_mcp']
      vars:
        docker_mcp_version: 'v0.40.0'
```

### Install for a specific user

```yaml
- name: Install Docker MCP CLI plugin for specific user
  hosts: all
  become: true
  roles:
    - role: docker_mcp
      tags: ['docker_mcp']
      vars:
        docker_mcp_user: 'developer'
        docker_mcp_user_home: '/home/developer'
```

## What This Role Does

1. **Installs Go** - Required for building the plugin
2. **Installs build dependencies** - Git and Make
3. **Clones the repository** - Shallow clone of mcp-gateway
4. **Creates plugins directory** - `~/.docker/cli-plugins/`
5. **Builds the plugin** - Runs `make docker-mcp`
6. **Verifies installation** - Confirms plugin exists
7. **Cleans up** - Removes temporary build directory

## Secrets in Devcontainers

To use `docker mcp gateway run` with Docker Desktop secrets inside a devcontainer (Linux), also install the [`docker_pass`](../docker_pass/README.md) role. Without it, the gateway cannot resolve secrets from Docker Desktop's store because the macOS Keychain is not accessible from Linux.

## Verification

After running the role, verify the installation:

```bash
docker mcp --help
```

## References

- [Docker MCP Gateway Documentation](https://docs.docker.com/ai/mcp-gateway)
- [docker/mcp-gateway GitHub Repository](https://github.com/docker/mcp-gateway)
