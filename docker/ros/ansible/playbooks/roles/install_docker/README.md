# Install Docker Role

This Ansible role installs and configures Docker Engine on Ubuntu systems,
including setting up the repository, installing Docker components,
configuring the daemon, and adding the user to the `docker` group.

## Example Usage

```yaml
- name: Install Docker
  hosts: all
  become: true
  roles:
    - { role: install_docker, tags: ["install_docker"] }
```

## Notes

This role is intentionally focused on core installation only. Service startup
is handled by the `install_docker_service` role where systemd is available.
