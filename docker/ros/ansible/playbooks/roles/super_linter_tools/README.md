# Super Linter Tools Role

Installs the linter toolchain used by this repository's local super-linter wrapper:

- prettier (Node/Bun global package)
- shellcheck (from distro apt; not version-pinned in this role)
- hadolint
- actionlint
- zizmor
- gitleaks

The role is designed for amd64 and arm64 and is invoked from
`docker/ros/ansible/playbooks/20_ros_setup.yml`.
