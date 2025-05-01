# Locale Setup Role

This Ansible role configures the system locale to en_US.UTF-8, which is required for proper functioning of ROS 2.

## Example Usage

```yaml
- name: Configure UTF-8 locale
  hosts: all
  become: true
  roles:
    - { role: locale_setup, tags: ["locale_setup"] }
```

## Notes

Proper locale configuration is essential for ROS 2 and many other applications to function correctly. This role ensures that the system is using UTF-8 encoding, which is required by ROS 2.
