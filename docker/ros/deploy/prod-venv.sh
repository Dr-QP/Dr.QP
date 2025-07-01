#!/usr/bin/env bash

script_dir=$(dirname $0)

python3 -m venv .venv-prod --system-site-packages
source .venv-prod/bin/activate
python3 -m pip install -r $script_dir/prod-requirements.txt
