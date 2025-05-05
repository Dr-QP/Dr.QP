# Fish Shell Setup Role

This Ansible role sets up the fish shell for ROS 2 development. It installs the fish package manager (fisher), bass for sourcing bash scripts, and configures ROS 2 functions for convenient usage.

## Example Usage

```yaml
- name: Setup fish shell for ROS 2
  hosts: all
  roles:
    - { role: fish_setup, tags: ["fish_setup"] }
```
