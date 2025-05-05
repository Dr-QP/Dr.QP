# Install Docker Role

This Ansible role installs and configures Docker Engine on Ubuntu systems, including setting up the repository, installing Docker components, configuring the daemon, and adding the user to the docker group.

## Example Usage

```yaml
- name: Install Docker
  hosts: all
  become: true
  roles:
    - { role: install_docker, tags: ["install_docker"] }
```

## Notes

This role is used by the `10_install_docker.yml` playbook and the `100_startup_service.yml` playbook for setting up Docker-based ROS 2 services.
