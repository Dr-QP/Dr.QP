# Install Docker Service Role

This Ansible role manages Docker services via systemd:

- `docker.service`
- `containerd.service`

Use this role together with `install_docker` on hosts where systemd is
available.
