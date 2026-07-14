#!/usr/bin/env bash

script_dir=$(dirname "${BASH_SOURCE[0]}")
# __utils.sh derives and exports root_dir for this script.
# shellcheck disable=SC1091
source "$script_dir/__utils.sh"

# shellcheck source=/dev/null
# root_dir is exported by __utils.sh above.
# shellcheck disable=SC2154
source "$root_dir/.venv/bin/activate"

jupytext --sync "$root_dir/docs/source/notebooks/*.md" "$root_dir/docs/source/notebooks/*.ipynb" --quiet "$@"
