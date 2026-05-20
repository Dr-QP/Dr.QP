#!/usr/bin/env bash
set -euo pipefail

search_roots=("$@")
if [[ ${#search_roots[@]} -eq 0 ]]; then
  search_roots=(build install)
fi

pip_install=(python3 -m pip install --break-system-packages)
if [[ $(id -u) -ne 0 ]]; then
  pip_install=(sudo "${pip_install[@]}")
fi

for root in "${search_roots[@]}"; do
  [[ -d "$root" ]] || continue

  while IFS= read -r -d '' path; do
    mapfile -t requirements < <(
      python3 - "$path" <<'PY'
import sys
from pathlib import Path

for raw_line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines():
    line = raw_line.strip()
    if not line or line.startswith("#") or line.startswith("["):
        continue
    print(line)
PY
    )

    if [[ ${#requirements[@]} -eq 0 ]]; then
      continue
    fi

    echo "Installing requirements from $path"
    "${pip_install[@]}" "${requirements[@]}"
  done < <(find "$root" -name requires.txt -print0)
done
