# Development Tools Role

This Ansible role installs a comprehensive set of development tools required for ROS 2 development, including build tools, version control, Python tools, and required libraries.

## Example Usage

```yaml
- name: Install development tools
  hosts: all
  become: true
  roles:
    - { role: dev_tools, tags: ["dev_tools"] }
```

## Notes

This role installs all the necessary development tools for building and testing ROS 2 packages. It's a prerequisite for both source and binary installations of ROS 2.
