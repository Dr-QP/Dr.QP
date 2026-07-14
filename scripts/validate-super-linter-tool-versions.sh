#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
pre_commit_config="$root_dir/.pre-commit-config.yaml"
zizmor_hook="$script_dir/zizmor-pre-commit.sh"
. "$script_dir/super-linter-defaults.sh"

usage()
{
  cat <<'EOF'
Usage: scripts/validate-super-linter-tool-versions.sh [--image IMAGE]

Validates that the versions used by pre-commit and local hooks match the
versions installed in the pinned Super-Linter image. Super-Linter is the
source of truth.

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

if command -v docker >/dev/null 2>&1; then
  runtime=docker
elif command -v podman >/dev/null 2>&1; then
  runtime=podman
else
  echo "Docker or Podman is required to inspect Super-Linter." >&2
  exit 1
fi

pre_commit_rev()
{
  local repository="$1"

  awk -v repository="$repository" '
    $0 ~ /^  - repo: / {
      if (found) {
        exit
      }
      found = $3 == repository
      next
    }
    found && $0 ~ /^    rev: / {
      print $2
      exit
    }
  ' "$pre_commit_config"
}

normalize_configured_version()
{
  local tool="$1"
  local version="${2#v}"

  # The ShellCheck pre-commit wrapper has a fourth patch component (for
  # example, 0.11.0.1) while the ShellCheck binary has three components.
  if [[ "$tool" == shellcheck ]]; then
    version="${version%.*}"
  fi

  printf '%s\n' "$version"
}

check_version()
{
  local tool="$1"
  local configured
  local super_linter

  configured=$(normalize_configured_version "$tool" "${configured_versions[$tool]}")
  super_linter="${super_linter_versions[$tool]#v}"

  if [[ -z "$configured" || -z "$super_linter" ]]; then
    echo "Could not determine the $tool version." >&2
    return 1
  fi

  if [[ "$configured" != "$super_linter" ]]; then
    printf 'Version mismatch for %s: configured %s, Super-Linter %s\n' \
      "$tool" "$configured" "$super_linter" >&2
    return 1
  fi

  printf 'OK %-14s %s\n' "$tool" "$super_linter"
}

check_configured_version()
{
  local label="$1"
  local tool="$2"
  local configured="$3"
  local super_linter

  configured=$(normalize_configured_version "$tool" "$configured")
  super_linter="${super_linter_versions[$tool]#v}"

  if [[ "$configured" != "$super_linter" ]]; then
    printf 'Version mismatch for %s: configured %s, Super-Linter %s\n' \
      "$label" "$configured" "$super_linter" >&2
    return 1
  fi

  printf 'OK %-14s %s\n' "$label" "$super_linter"
}

# shellcheck disable=SC2016
super_linter_output=$("$runtime" run --rm --entrypoint /bin/bash "$image" -c '
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

declare -A super_linter_versions
while IFS='=' read -r tool version; do
  super_linter_versions["$tool"]="$version"
done <<< "$super_linter_output"

declare -A configured_versions
configured_versions[prettier]=$(sed -nE \
  's/.*prettier@v?([0-9.]+).*/\1/p' "$pre_commit_config" | head -n 1)
configured_versions[clang-format]=$(pre_commit_rev \
  https://github.com/pre-commit/mirrors-clang-format)
configured_versions[ansible-lint]=$(pre_commit_rev https://github.com/ansible/ansible-lint)
configured_versions[hadolint]=$(pre_commit_rev https://github.com/hadolint/hadolint)
configured_versions[ruff]=$(pre_commit_rev https://github.com/astral-sh/ruff-pre-commit)
configured_versions[shellcheck]=$(pre_commit_rev https://github.com/shellcheck-py/shellcheck-py)
configured_versions[gitleaks]=$(pre_commit_rev https://github.com/gitleaks/gitleaks)
configured_versions[actionlint]=$(pre_commit_rev https://github.com/rhysd/actionlint)
configured_versions[zizmor]=$(sed -nE 's/^ZIZMOR_VERSION="?([^" ]+)"?/\1/p' \
  "$zizmor_hook" | head -n 1)

for tool in prettier clang-format ansible-lint hadolint ruff shellcheck gitleaks actionlint zizmor; do
  check_version "$tool"
done

hadolint_entry_version=$(sed -nE \
  's/.*ghcr\.io\/hadolint\/hadolint:v?([0-9.]+).*/\1/p' "$pre_commit_config" | head -n 1)
check_configured_version "hadolint entry" hadolint "$hadolint_entry_version"

super_linter_tag="v${image##*:v}"
while IFS= read -r configured_tag; do
  if [[ "$configured_tag" != "$super_linter_tag" ]]; then
    printf 'Version mismatch for Super-Linter: local image %s, workflow %s\n' \
      "$super_linter_tag" "$configured_tag" >&2
    exit 1
  fi
done < <(sed -nE 's/.*super-linter\/super-linter@(v[0-9.]+).*/\1/p' \
  "$root_dir/.github/workflows/reformat.yml")

echo "All pre-commit and local tool versions match $image."
