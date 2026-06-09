#!/usr/bin/env bash
set -euo pipefail

ANSIBLE_VERSION="${ANSIBLE_VERSION:-13.4.0}"

echo "Installing Ansible ${ANSIBLE_VERSION} from PyPI..."

python3 -m pip install \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    "ansible==${ANSIBLE_VERSION}"

python3 - <<'PY'
from ansible.plugins.action.include_vars import ActionModule
import ansible.release

print(f"Verified ansible {ansible.release.__version__}: include_vars import ok")
PY

# Install required Ansible collections
if ! ansible-galaxy collection list | grep -q "community.general"; then
    echo "Installing required Ansible collections..."
    ansible-galaxy collection install community.general
fi
