# Bash Setup Role

This Ansible role configures Bash shell for ROS 2 development by adding convenient aliases to the user's `.bashrc` file.

## Example Usage

```yaml
- name: Setup Bash shell for ROS 2
  hosts: all
  roles:
    - { role: bash_setup, tags: ["bash_setup"] }
```
