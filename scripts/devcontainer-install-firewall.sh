#!/usr/bin/env bash
# Runs the devcontainer firewall Ansible playbook to install iptables/ipset and
# the init-firewall.sh script.  Called once from postCreateCommand so the tools
# are in place before the first postStartCommand run.
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

ansible-playbook \
    -i "$repo_root/docker/ros/ansible/inventories/localhost.yml" \
    "$repo_root/docker/ros/ansible/playbooks/25_devcontainer_firewall.yml"
