#!/usr/bin/env bash
set -euo pipefail

ANSIBLE_VERSION="${ANSIBLE_VERSION:-13.4.0}"
SYSTEM_PYTHON="/usr/bin/python3"

pip_install=("${SYSTEM_PYTHON}" -m pip install)
if [[ $(id -u) -ne 0 ]]; then
    if ! command -v sudo >/dev/null 2>&1; then
        echo "Error: root privileges required to install Ansible system-wide; sudo is not available." >&2
        exit 1
    fi
    pip_install=(sudo "${pip_install[@]}")
fi

echo "Installing Ansible ${ANSIBLE_VERSION} from PyPI..."

"${pip_install[@]}" \
    --break-system-packages \
    --ignore-installed \
    --no-cache-dir \
    "ansible==${ANSIBLE_VERSION}"

"${SYSTEM_PYTHON}" - <<'PY'
from ansible.plugins.action.include_vars import ActionModule
import ansible.release

print(f"Verified ansible {ansible.release.__version__}: include_vars import ok")
PY

# Install required Ansible collections
if ! "${SYSTEM_PYTHON}" -m ansible.cli.galaxy collection list | grep -q "community.general"; then
    echo "Installing required Ansible collections..."
    "${SYSTEM_PYTHON}" -m ansible.cli.galaxy collection install community.general
fi
