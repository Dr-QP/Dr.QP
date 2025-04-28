#!/usr/bin/env bash
set -e

# Check if Ansible is installed
if ! command -v ansible-playbook &> /dev/null; then
    echo "Ansible is not installed. Installing..."
    sudo apt-get update
    sudo apt-get install -y software-properties-common
    sudo apt-add-repository --yes --update ppa:ansible/ansible
    sudo apt-get install -y ansible
fi

# Install required Ansible collections
if ! ansible-galaxy collection list | grep -q "community.general"; then
    echo "Installing required Ansible collections..."
    ansible-galaxy collection install community.general
fi
