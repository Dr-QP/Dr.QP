#!/usr/bin/env bash

script_dir=$(dirname "${BASH_SOURCE[0]}")
# __utils.sh derives and exports root_dir for this script.
# shellcheck disable=SC1091
source "$script_dir/__utils.sh"

"$root_dir/.venv/bin/jupytext" --sync "$root_dir/docs/source/notebooks/*.md" "$root_dir/docs/source/notebooks/*.ipynb" --quiet "$@"
