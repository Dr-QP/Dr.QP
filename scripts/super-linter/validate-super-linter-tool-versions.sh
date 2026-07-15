#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/../.." && pwd)
pre_commit_config="$root_dir/.pre-commit-config.yaml"
. "$script_dir/super-linter-defaults.sh"

usage()
{
  cat <<'EOF'
Usage: scripts/validate-super-linter-tool-versions.sh [--image IMAGE]

Validates that the pre-commit helpers and GitHub Actions use the pinned
Super-Linter image. Super-Linter is the source of truth for these tools.

Options:
  --image IMAGE  Override the Super-Linter image to inspect.
  -h, --help     Show this help.

Environment:
  SUPER_LINTER_IMAGE  Override the Super-Linter image to inspect.
EOF
}

image="${SUPER_LINTER_IMAGE:-$SUPER_LINTER_DEFAULT_IMAGE}"
while (($#)); do
  case "$1" in
    --image)
      if [[ $# -lt 2 ]]; then
        echo "--image requires a value." >&2
        exit 2
      fi
      image="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done


# shellcheck disable=SC2016
super_linter_output=$(SUPER_LINTER_IMAGE="$image" "$script_dir/super-linter-run.sh" bash -c '
set -euo pipefail
printf "prettier=%s\n" "$(prettier --version)"
printf "clang-format=%s\n" "$(clang-format --version | sed -nE "s/.*version ([0-9.]+).*/\\1/p")"
printf "ansible-lint=%s\n" "$(ansible-lint --version | awk "NR == 1 { print \$2 }")"
printf "hadolint=%s\n" "$(hadolint --version | awk "NR == 1 { print \$NF }")"
printf "ruff=%s\n" "$(ruff --version | awk "NR == 1 { print \$2 }")"
printf "shellcheck=%s\n" "$(shellcheck --version | sed -nE "s/^version: ([0-9.]+).*/\\1/p")"
printf "gitleaks=%s\n" "$(gitleaks version | awk "NR == 1 { print \$1 }")"
printf "actionlint=%s\n" "$(actionlint --version | head -n 1)"
printf "zizmor=%s\n" "$(zizmor --version | awk "NR == 1 { print \$2 }")"
')

validation_failed=0

for tool in prettier clang-format ansible-lint hadolint ruff shellcheck gitleaks actionlint zizmor; do
  version=$(sed -nE "s/^${tool}=//p" <<< "$super_linter_output" | head -n 1)
  if [[ -z "$version" ]]; then
    echo "Could not determine the Super-Linter $tool version." >&2
    validation_failed=1
    continue
  fi

  if ! grep -Fq "        entry: ./scripts/super-linter/$tool" "$pre_commit_config"; then
    echo "Pre-commit does not use the $tool Super-Linter helper." >&2
    validation_failed=1
    continue
  fi

  printf 'OK %-14s %s\n' "$tool" "$version"
done

super_linter_tag="v${image##*:v}"
workflow_super_linter_tags=$(sed -nE \
  's/.*super-linter\/super-linter@(v[0-9.]+).*/\1/p' \
  "$root_dir/.github/workflows/reformat.yml")

if [[ -z "$workflow_super_linter_tags" ]]; then
  echo "Could not find a recognized Super-Linter version tag in the workflow." >&2
  validation_failed=1
else
  while IFS= read -r configured_tag; do
    if [[ "$configured_tag" != "$super_linter_tag" ]]; then
      printf 'Version mismatch for Super-Linter: local image %s, workflow %s\n' \
        "$super_linter_tag" "$configured_tag" >&2
      validation_failed=1
    fi
  done <<< "$workflow_super_linter_tags"
fi

if ((validation_failed)); then
  exit 1
fi

echo "All pre-commit helpers use tools from $image."
