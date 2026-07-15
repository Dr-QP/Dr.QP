#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)
. "$script_dir/super-linter-defaults.sh"

image="${SUPER_LINTER_IMAGE:-$SUPER_LINTER_DEFAULT_IMAGE}"
validate_all_codebase="${VALIDATE_ALL_CODEBASE:-false}"
log_level="${LOG_LEVEL:-INFO}"

usage()
{
  cat <<'EOF'
Usage: scripts/super-linter-local.sh [--all] [--image IMAGE] [--log-level LEVEL]

Runs one local Super-Linter pass with all checks enabled and available
autofixes applied. CI keeps its autofix and check passes separate so it can
publish formatter patches without failing the validation job.

Options:
  --all              Set VALIDATE_ALL_CODEBASE=true.
  --image IMAGE      Override the Super-Linter container image.
  --log-level LEVEL  Override Super-Linter LOG_LEVEL.
  -h, --help         Show this help.

Environment:
  SUPER_LINTER_IMAGE     Container image to run.
  VALIDATE_ALL_CODEBASE  true or false. Defaults to false.
  LOG_LEVEL              Super-Linter log level. Defaults to INFO.
EOF
}

while (($#)); do
  case "$1" in
    --all)
      validate_all_codebase=true
      shift
      ;;
    --image)
      if [[ $# -lt 2 ]]; then
        echo "--image requires a value." >&2
        exit 2
      fi
      image="$2"
      shift 2
      ;;
    --log-level)
      if [[ $# -lt 2 ]]; then
        echo "--log-level requires a value." >&2
        exit 2
      fi
      log_level="$2"
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
  echo "Docker or Podman is required to run Super-Linter locally." >&2
  exit 1
fi

mkdir -p "$root_dir/.tmp"

default_branch=$(git -C "$root_dir" symbolic-ref --quiet --short refs/remotes/origin/HEAD 2>/dev/null || true)
default_branch="${default_branch#origin/}"
if [[ -z "$default_branch" ]]; then
  default_branch=main
fi

git_common_dir=$(git -C "$root_dir" rev-parse --path-format=absolute --git-common-dir)
mounts=(
  -v "$root_dir:/tmp/lint"
)
if [[ "$git_common_dir" != "$root_dir/.git" ]]; then
  mounts+=(-v "$git_common_dir:$git_common_dir")
fi

run_super_linter() {
  local env_file="$1"

  "$runtime" run --rm \
  -e RUN_LOCAL=true \
  -e DEFAULT_BRANCH="$default_branch" \
  -e VALIDATE_ALL_CODEBASE="$validate_all_codebase" \
  -e LOG_LEVEL="$log_level" \
  -e SAVE_SUPER_LINTER_OUTPUT=true \
  -e SUPER_LINTER_OUTPUT_DIRECTORY_NAME="log" \
  --env-file "$env_file" \
  "${mounts[@]}" \
  --platform linux/amd64 \
  "$image"
}

env_file="$root_dir/.tmp/super-linter.env"
"$script_dir/super-linter-env.sh" --autofix --enable_all_checks > "$env_file"
run_super_linter "$env_file"

echo "Super-Linter output saved to $root_dir/log/super-linter-summary.md"
