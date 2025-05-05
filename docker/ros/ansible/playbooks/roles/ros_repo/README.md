# ROS Repository Role

This Ansible role sets up the ROS 2 repository for package installation by adding the signing key and repository sources.

## Example Usage

```yaml
- name: Setup ROS 2 repository
  hosts: all
  become: true
  roles:
    - { role: ros_repo, tags: ["ros_repo"] }
```

## Notes

This role is a prerequisite for installing ROS 2 packages from the official ROS 2 repositories. It sets up the repository for the appropriate architecture and Ubuntu distribution.
