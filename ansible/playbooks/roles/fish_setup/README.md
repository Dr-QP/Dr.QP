# Fish Shell Setup Role

This Ansible role sets up the fish shell for ROS 2 development.

## Role Tasks

This role performs the following tasks:

1. Installs fisher (fish package manager)
2. Installs bass (for sourcing bash scripts in fish)
3. Configures fish with ROS 2 functions:
   - `ros2_activate` - Sources the ROS 2 setup script
   - `ros2_ws` - Sources the workspace setup script
   - `register_argcomplete` - Sets up tab completion for ROS 2 commands

## Example Usage

```yaml
- name: Setup fish shell for ROS 2
  hosts: all
  roles:
    - { role: fish_setup, tags: ["fish_setup"] }
```
