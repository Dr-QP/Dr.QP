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

## Role variables

- `install_docker_manage_service` (default: `true`): start/restart
  `docker.service` and `containerd.service` via systemd.
- `install_docker_add_user_to_group` (default: `true`): add a user to the
  `docker` group.
- `install_docker_user` (default: `ansible_user` fallback): user to add to
  the `docker` group.

For container image builds that do not run systemd, set:

```yaml
install_docker_manage_service: false
install_docker_add_user_to_group: false
```
