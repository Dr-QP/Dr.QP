#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(dirname "$script_dir")

command=("$@")
set --

source "$root_dir/scripts/setup.bash"

exec "${command[@]}"
