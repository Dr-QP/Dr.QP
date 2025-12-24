#!/usr/bin/env bash

install_dir=${1}
script_dir=$(dirname $0)

python3 -m venv .venv-prod --system-site-packages
source .venv-prod/bin/activate

if [[ -n $install_dir ]]; then
  echo "Installing requirements from $install_dir"
  readarray -t requires < <(find "$install_dir" -name requires.txt)

  for path in "${requires[@]}"; do
      python3 -m pip install -r "$path"
  done
fi
