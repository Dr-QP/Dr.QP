# Colcon Setup Role

This Ansible role configures colcon for ROS 2 development by setting up mixins and metadata from the official repositories.

## Example Usage

```yaml
- name: Setup colcon
  hosts: all
  roles:
    - { role: colcon_setup, tags: ["colcon_setup"] }
```

## Notes

Colcon mixins provide predefined build configurations that can be reused across projects. For example, the project uses mixins like `coverage-pytest` and `ninja` for building and testing.
