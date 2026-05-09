#!/usr/bin/env bash

script_dir=$(dirname "${BASH_SOURCE[0]}")
root_dir_default="$(dirname "$(dirname "$(dirname "$script_dir")")")"
root_dir=${1:-$root_dir_default}

if [[ -f "$root_dir/.venv-prod/bin/activate" ]]; then
  exit 0
fi

python3 -m venv "$root_dir/.venv-prod" --system-site-packages
