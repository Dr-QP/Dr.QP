# Xpra Setup Role

This Ansible role installs Xpra, xpra-html5, VirtualGL, and related dependencies for remote desktop and application streaming on Ubuntu 24.04 (Noble).

## Overview

Xpra is a persistent remote application server that allows you to run X11 applications on a remote host and display them locally. This role sets up:

- **Xpra**: Remote desktop and application streaming server
- **xpra-html5**: HTML5 client for web-based access
- **VirtualGL**: GPU acceleration for remote rendering
- **Mesa utilities**: Software rendering fallback (llvmpipe)
- **Xvfb**: Virtual framebuffer for headless operation

## Requirements

- Ubuntu 24.04 (Noble) or compatible Debian-based distribution
- Internet access to download packages from official repositories
- Root or sudo access

## Role Variables

### `install_xpra`
- **Type**: Boolean
- **Default**: `false`
- **Description**: Enable or disable the installation of Xpra and related packages

## Example Usage

```yaml
- name: Install Xpra
  hosts: all
  become: true
  vars:
    install_xpra: true
  roles:
    - { role: xpra_setup, tags: ["xpra_setup"] }
```

Or with conditional installation:

```yaml
- name: Setup Docker image with optional Xpra
  hosts: all
  become: true
  roles:
    - { role: xpra_setup, tags: ["xpra_setup"] }
```

Then enable at runtime:
```bash
ansible-playbook playbook.yml -e "install_xpra=true"
```

## What Gets Installed

### From Official Xpra Installation Script
- `xpra` - Main Xpra server and client (installed via official `get-xpra.sh` script)
- `xpra-html5` - HTML5 web client (installed separately via APT)

The official Xpra installation script (`https://xpra.org/get-xpra.sh`) handles:
- Adding the Xpra GPG key
- Adding the correct APT repository for the current distribution
- Updating the APT cache
- Installing the base `xpra` package

### From GitHub Releases
- `virtualgl` - Latest version (auto-detected and downloaded)

### From Ubuntu Repositories
- `mesa-utils` - OpenGL utilities
- `libgl1` - OpenGL runtime
- `libglvnd0` - OpenGL vendor library
- `xvfb` - Virtual framebuffer
- `libgl1-mesa-dri` - Software rendering support (Mesa DRI with llvmpipe)

## Idempotency

This role is idempotent. Running it multiple times will not cause issues:
- Repository files are overwritten (safe)
- Package installations are idempotent
- VirtualGL version detection ensures correct package is installed

## Architecture Support

The role automatically detects the system architecture and downloads the appropriate VirtualGL package:
- **amd64** (x86_64)
- **arm64** (aarch64)

## Notes

- The role requires internet access to download packages from official repositories
- VirtualGL version is auto-detected from GitHub releases
- The role uses `install_recommends: false` to minimize image size
- All tasks are conditional on `install_xpra` variable for flexibility

