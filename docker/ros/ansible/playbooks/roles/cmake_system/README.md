# CMake System Role

This Ansible role installs CMake from the standard Ubuntu repositories, removing any Kitware CMake installation if present.

## Example Usage

```yaml
- name: Install system CMake
  hosts: all
  become: true
  roles:
    - { role: cmake_system, tags: ["cmake"] }
```

## Notes

This role is used when installing ROS 2 from source, as it ensures compatibility with the ROS 2 build process.
