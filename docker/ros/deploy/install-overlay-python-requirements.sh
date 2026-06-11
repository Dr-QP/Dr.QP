#!/usr/bin/env bash
set -euo pipefail

search_roots=("$@")
if [[ ${#search_roots[@]} -eq 0 ]]; then
  search_roots=(build install)
fi

pip_install=(/usr/bin/python3 -m pip install --break-system-packages)
if [[ $(id -u) -ne 0 ]]; then
  pip_install=(sudo "${pip_install[@]}")
fi

filtered_requirements_file=""

cleanup_filtered_requirements_file() {
  if [[ -n "${filtered_requirements_file:-}" ]]; then
    rm -f "$filtered_requirements_file"
    filtered_requirements_file=""
  fi
}

trap cleanup_filtered_requirements_file EXIT

install_requires_file() {
  local path="$1"
  local status

  # Filter out INI-style sections and non-package lines.
  # pip does not understand [sections], so strip them before install.
  filtered_requirements_file="$(mktemp -t filtered-requires.XXXXXX)"

  status=0
  grep -v -E '^\s*(#|\[|$)' "$path" > "$filtered_requirements_file" || status=$?
  if [[ $status -ne 0 && $status -ne 1 ]]; then
    return "$status"
  fi
  if [[ ! -s "$filtered_requirements_file" ]]; then
    cleanup_filtered_requirements_file
    return 0
  fi

  echo "Installing requirements from $path"
  "${pip_install[@]}" -r "$filtered_requirements_file"
  cleanup_filtered_requirements_file
}

for root in "${search_roots[@]}"; do
  [[ -d "$root" ]] || continue

  readarray -t requires < <(find "$root" -name requires.txt)

  for path in "${requires[@]}"; do
    install_requires_file "$path"
  done
done
