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
    - { role: install_docker, tags: ['install_docker'] }
```

## Notes

This role is intentionally focused on core installation only. Service startup
is handled by the `install_docker_service` role where systemd is available.

Package versions are pinned in `vars/apt_pins_<suite>_<arch>.yml`, resolved
against the two apt sources a robot host has — the Ubuntu archive and Docker's
own repository. The pins are specific to the Ubuntu release in the filename, so
a host on a different release needs its own pin file. See
[Pinned apt versions](../../../../../docs/source/GettingStarted/ansible.md#pinned-apt-versions).
