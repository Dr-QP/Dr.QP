#!/usr/bin/env bash

install_dir=${1}
script_dir=$(dirname $0)

python3 -m venv .venv-prod --system-site-packages
source .venv-prod/bin/activate

if [[ -n $install_dir ]]; then
  echo "Installing requirements from $install_dir"
  readarray -t requires < <(find "$install_dir" -name requires.txt)

  for path in "${requires[@]}"; do
      # Filter out INI-style sections and non-package lines.
      # pip does not understand [sections], so strip them before install.
      filtered="$(mktemp)"
      grep -v -E '^\s*(#|\[|$)' "$path" > "$filtered"
      if [[ -s "$filtered" ]]; then
        python3 -m pip install -r "$filtered"
      fi
      rm -f "$filtered"
  done
fi
