#!/usr/bin/env bash

root_dir=${1:-$PWD}

if [[ -f "$root_dir/.venv-prod/bin/activate" ]]; then
  exit 0
fi

python3 -m venv "$root_dir/.venv-prod" --system-site-packages
