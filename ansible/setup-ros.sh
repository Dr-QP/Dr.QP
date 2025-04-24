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

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Change to the script directory
cd "$SCRIPT_DIR"

# Run the Ansible playbook
ansible-playbook playbooks/20_ros_setup.yml "$@"

echo "ROS 2 setup completed successfully!"
