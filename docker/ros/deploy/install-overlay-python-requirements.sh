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

install_requires_file() {
  local path="$1"
  local filtered
  local status

  # Filter out INI-style sections and non-package lines.
  # pip does not understand [sections], so strip them before install.
  filtered="$(mktemp -t filtered-requires.XXXXXX)"
  trap 'rm -f "$filtered"' RETURN

  status=0
  grep -v -E '^\s*(#|\[|$)' "$path" > "$filtered" || status=$?
  if [[ $status -ne 0 && $status -ne 1 ]]; then
    return "$status"
  fi
  if [[ ! -s "$filtered" ]]; then
    return 0
  fi

  echo "Installing requirements from $path"
  "${pip_install[@]}" -r "$filtered"
}

for root in "${search_roots[@]}"; do
  [[ -d "$root" ]] || continue

  readarray -t requires < <(find "$root" -name requires.txt)

  for path in "${requires[@]}"; do
    install_requires_file "$path"
  done
done
