# Basic Prerequisites Role

This Ansible role installs basic prerequisites required for ROS 2 development, including essential packages and enables the Ubuntu universe repository.

## Example Usage

```yaml
- name: Install basic prerequisites
  hosts: all
  become: true
  roles:
    - { role: basic_prereqs, tags: ["basic_prereqs"] }
```
