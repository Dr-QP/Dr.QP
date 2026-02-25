#!/usr/bin/env bash

root_dir=${PWD}

if [[ -d "$root_dir/.venv-prod" ]]; then
  exit 0
fi

python3 -m venv "$root_dir/.venv-prod" --system-site-packages
