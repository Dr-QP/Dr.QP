#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)

image="${SUPER_LINTER_IMAGE:-ghcr.io/super-linter/super-linter:v8.5.0}"
validate_all_codebase="${VALIDATE_ALL_CODEBASE:-false}"
log_level="${LOG_LEVEL:-INFO}"

usage()
{
  cat <<'EOF'
Usage: scripts/super-linter-local.sh [--all] [--image IMAGE] [--log-level LEVEL]

Runs the same Super-Linter autofix pass and check pass used by GitHub Actions.

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
mounts=(-v "$root_dir:/tmp/lint")
if [[ "$git_common_dir" != "$root_dir/.git" ]]; then
  mounts+=(-v "$git_common_dir:$git_common_dir")
fi

run_super_linter()
{
  local name="$1"
  local env_file="$2"

  echo "Running Super-Linter $name pass..."
  "$runtime" run --rm \
    -e RUN_LOCAL=true \
    -e DEFAULT_BRANCH="$default_branch" \
    -e VALIDATE_ALL_CODEBASE="$validate_all_codebase" \
    -e LOG_LEVEL="$log_level" \
    --env-file "$root_dir/$env_file" \
    "${mounts[@]}" \
    "$image"
}

run_super_linter "autofix" ".github/super-linter-autofix.env"
run_super_linter "check" ".github/super-linter-checks.env"
