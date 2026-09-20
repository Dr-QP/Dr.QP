# CMake System Role

This Ansible role installs CMake from the standard Ubuntu repositories, removing any Kitware CMake installation if present.

## Example Usage

```yaml
- name: Install system CMake
  hosts: all
  become: true
  roles:
    - { role: cmake_system, tags: ['cmake'] }
```

## Notes

This role is used when installing ROS 2 from source, as it ensures compatibility with the ROS 2 build process.

The CMake version is pinned in `vars/apt_pins_<suite>_<arch>.yml`. That pin is resolved against the Ubuntu archive alone — the role removes the Kitware repository before installing, so a Kitware version would no longer be fetchable by the time apt runs. See [Pinned apt versions](../../../../../docs/source/GettingStarted/ansible.md#pinned-apt-versions).
