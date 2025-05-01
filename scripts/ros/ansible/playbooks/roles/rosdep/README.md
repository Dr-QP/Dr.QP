# Rosdep Role

This Ansible role initializes and updates rosdep, the ROS dependency management tool used for installing system dependencies for ROS packages.

## Example Usage

```yaml
- name: Setup rosdep
  hosts: all
  become: true
  roles:
    - { role: rosdep, tags: ["rosdep"] }
```

## Notes

Rosdep is a command-line tool for installing system dependencies for ROS packages. This role ensures that rosdep is properly initialized and updated, which is required for installing ROS packages and their dependencies.
