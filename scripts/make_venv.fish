#!/usr/bin/env fish

python3 -m venv .venv
source .venv/bin/activate.fish
python3 -m pip install -r requirements.txt -r docs/requirements.txt
