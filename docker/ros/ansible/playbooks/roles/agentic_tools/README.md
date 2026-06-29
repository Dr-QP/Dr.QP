# Agentic Tools Role

Installs the agentic CLI tooling used in the workspace and the security layer
that guards it:

- **Bun-managed globals**: `@modelcontextprotocol/inspector`, `@github/copilot`,
  and `@anthropic-ai/claude-code`.
- **[cc-filter](https://github.com/wissem/cc-filter)**: a hard security layer in
  front of Claude Code hooks. It blocks sensitive file access, blocks risky
  shell/search commands, and redacts secrets. The role downloads the
  architecture-appropriate release binary and wires it into the user's Claude
  Code hooks (`~/.claude/settings.json`).

Gated by `install_agentic_tools` in the playbook.

## Example Usage

```yaml
- name: Install agentic tools
  hosts: all
  become: true
  roles:
    - {
        role: agentic_tools,
        tags: ['agentic_tools'],
        when: install_agentic_tools | default(false) | bool,
      }
```

## Variables

| Variable                    | Default                    | Description                                        |
| --------------------------- | -------------------------- | -------------------------------------------------- |
| `cc_filter_install`         | `true`                     | Install the cc-filter binary.                      |
| `cc_filter_version`         | `v0.0.6`                   | cc-filter release tag to download.                 |
| `cc_filter_install_path`    | `/usr/local/bin/cc-filter` | Destination for the cc-filter binary.              |
| `cc_filter_binary_mode`     | `"0755"`                   | File mode for the installed binary.                |
| `cc_filter_checksums`       | per-arch sha256 map        | Expected binary checksums; bump with the version.  |
| `cc_filter_configure_hooks` | `true`                     | Merge cc-filter hooks into Claude `settings.json`. |
| `cc_filter_download_url`    | derived from version/arch  | Override to pin a custom binary URL.               |

## Required External Variables

These variables are not defined by this role and must be supplied by the
playbook or inventory (the `extra_facts` and `ros_user_setup` roles provide them
in this repository):

| Variable                                | Description                                                      |
| --------------------------------------- | ---------------------------------------------------------------- |
| `system_arch`                                | Target CPU architecture (`amd64` or `arm64`) for the binary URL.       |
| `user_home`                                  | Home directory of the target user (locates `~/.claude/settings.json`). |
| `ros_user_setup_uid` / `ros_user_setup_gid`  | UID/GID used for ownership of the Claude config files.                 |
