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

for root in "${search_roots[@]}"; do
  [[ -d "$root" ]] || continue

  readarray -t requires < <(find "$root" -name requires.txt)

  for path in "${requires[@]}"; do
    filtered="$(mktemp)"
    grep -v -E '^\s*(#|\[|$)' "$path" > "$filtered"
    if [[ ! -s "$filtered" ]]; then
      rm -f "$filtered"
      continue
    fi

    echo "Installing requirements from $path"
    "${pip_install[@]}" -r "$filtered"
    rm -f "$filtered"
  done
done
