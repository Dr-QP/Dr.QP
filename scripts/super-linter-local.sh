#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root_dir=$(cd "$script_dir/.." && pwd)

image="${SUPER_LINTER_IMAGE:-ghcr.io/super-linter/super-linter:v8.5.0}"
validate_all_codebase="${VALIDATE_ALL_CODEBASE:-false}"
log_level="${LOG_LEVEL:-INFO}"
mode="${SUPER_LINTER_MODE:-native}"
filter_regex_exclude='(^|/)(build|install|log|\.venv|packages/vendor|\.git)/'

usage()
{
  cat <<'EOF'
Usage: scripts/super-linter-local.sh [--all] [--native|--container|--auto] [--image IMAGE] [--log-level LEVEL]

Runs the same lint set as CI with a native-first workflow.

Modes:
  --native           Run linters installed in the devcontainer (default).
  --container        Force the Super-Linter container workflow.
  --auto             Prefer native tools, fall back to Super-Linter container when missing.

Options:
  --all              Set VALIDATE_ALL_CODEBASE=true.
  --native           Same as SUPER_LINTER_MODE=native.
  --container        Same as SUPER_LINTER_MODE=container.
  --auto             Same as SUPER_LINTER_MODE=auto.
  --image IMAGE      Override the Super-Linter container image.
  --log-level LEVEL  Override Super-Linter LOG_LEVEL.
  -h, --help         Show this help.

Environment:
  SUPER_LINTER_IMAGE     Container image to run.
  VALIDATE_ALL_CODEBASE  true or false. Defaults to false.
  LOG_LEVEL              Super-Linter log level. Defaults to INFO.
  SUPER_LINTER_MODE      native, container, or auto. Defaults to native.
EOF
}

while (($#)); do
  case "$1" in
    --all)
      validate_all_codebase=true
      shift
      ;;
    --native)
      mode=native
      shift
      ;;
    --container)
      mode=container
      shift
      ;;
    --auto)
      mode=auto
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

mkdir -p "$root_dir/.tmp"

default_branch=$(git -C "$root_dir" symbolic-ref --quiet --short refs/remotes/origin/HEAD 2>/dev/null || true)
default_branch="${default_branch#origin/}"
if [[ -z "$default_branch" ]]; then
  default_branch=main
fi

collect_target_files()
{
  local files_list="$root_dir/.tmp/super-linter-files.$$.txt"
  : > "$files_list"

  if [[ "$validate_all_codebase" == true ]]; then
    git -C "$root_dir" ls-files > "$files_list"
  else
    {
      if git -C "$root_dir" show-ref --verify --quiet "refs/remotes/origin/$default_branch"; then
        git -C "$root_dir" diff --name-only "origin/$default_branch"...HEAD
      fi

      git -C "$root_dir" diff --name-only
      git -C "$root_dir" diff --name-only --cached
      git -C "$root_dir" ls-files --others --exclude-standard
    } > "$files_list"
  fi

  sort -u "$files_list" -o "$files_list"
  mapfile -t all_files < "$files_list"
  rm -f "$files_list"
}

classify_files()
{
  prettier_files=()
  bash_files=()
  docker_files=()
  workflow_files=()

  for file in "${all_files[@]}"; do
    [[ -n "$file" ]] || continue
    [[ -f "$root_dir/$file" ]] || continue
    [[ "$file" =~ $filter_regex_exclude ]] && continue

    case "$file" in
      *.md|*.markdown|*.json|*.jsonc|*.yaml|*.yml)
        prettier_files+=("$file")
        ;;
    esac

    case "$file" in
      *.sh|*.bash|*.zsh)
        bash_files+=("$file")
        ;;
    esac

    case "$(basename "$file")" in
      Dockerfile|Dockerfile.*)
        docker_files+=("$file")
        ;;
    esac

    case "$file" in
      .github/workflows/*.yml|.github/workflows/*.yaml)
        workflow_files+=("$file")
        ;;
    esac
  done
}

run_in_root()
{
  (
    cd "$root_dir"
    "$@"
  )
}

ensure_native_tools()
{
  local missing=()
  local required=(prettier shellcheck hadolint actionlint zizmor gitleaks git)

  for tool in "${required[@]}"; do
    if ! command -v "$tool" >/dev/null 2>&1; then
      missing+=("$tool")
    fi
  done

  if ((${#missing[@]} > 0)); then
    echo "Missing native lint tools: ${missing[*]}" >&2
    echo "Install them by running the ROS setup playbook used by this repo images." >&2
    return 1
  fi

  return 0
}

run_native_super_linter()
{
  local status=0
  local prettier_log_level

  case "$log_level" in
    INFO)
      prettier_log_level=log
      ;;
    WARN|WARNING)
      prettier_log_level=warn
      ;;
    ERROR)
      prettier_log_level=error
      ;;
    DEBUG)
      prettier_log_level=debug
      ;;
    *)
      prettier_log_level=$(tr '[:upper:]' '[:lower:]' <<<"$log_level")
      ;;
  esac

  collect_target_files
  classify_files

  echo "Running native super-linter autofix pass..."
  if ((${#prettier_files[@]} > 0)); then
    run_in_root prettier --log-level "$prettier_log_level" --write "${prettier_files[@]}" || status=$?
  else
    echo "Skipping prettier autofix: no matching files."
  fi

  if ((${#workflow_files[@]} > 0)); then
    run_in_root zizmor --fix "${workflow_files[@]}" || status=$?
  else
    echo "Skipping zizmor autofix: no workflow files."
  fi

  echo "Running native super-linter check pass..."
  if ((${#prettier_files[@]} > 0)); then
    run_in_root prettier --log-level "$prettier_log_level" --check "${prettier_files[@]}" || status=$?
  else
    echo "Skipping prettier checks: no matching files."
  fi

  if ((${#bash_files[@]} > 0)); then
    run_in_root shellcheck "${bash_files[@]}" || status=$?
  else
    echo "Skipping shellcheck: no shell scripts."
  fi

  if ((${#docker_files[@]} > 0)); then
    run_in_root hadolint "${docker_files[@]}" || status=$?
  else
    echo "Skipping hadolint: no Dockerfiles."
  fi

  if ((${#workflow_files[@]} > 0)); then
    run_in_root actionlint "${workflow_files[@]}" || status=$?
    run_in_root zizmor "${workflow_files[@]}" || status=$?
  else
    echo "Skipping workflow checks: no workflow files."
  fi

  run_in_root gitleaks detect --source . --no-banner || status=$?
  return "$status"
}

run_super_linter()
{
  local name="$1"
  local env_file="$2"

  local runtime
  if command -v docker >/dev/null 2>&1; then
    runtime=docker
  elif command -v podman >/dev/null 2>&1; then
    runtime=podman
  else
    echo "Docker or Podman is required to run Super-Linter in container mode." >&2
    return 1
  fi

  local git_common_dir
  local -a mounts
  git_common_dir=$(git -C "$root_dir" rev-parse --path-format=absolute --git-common-dir)
  mounts=(-v "$root_dir:/tmp/lint")
  if [[ "$git_common_dir" != "$root_dir/.git" ]]; then
    mounts+=(-v "$git_common_dir:$git_common_dir")
  fi

  echo "Running Super-Linter $name pass..."
  "$runtime" run --rm \
    -e RUN_LOCAL=true \
    -e DEFAULT_BRANCH="$default_branch" \
    -e VALIDATE_ALL_CODEBASE="$validate_all_codebase" \
    -e LOG_LEVEL="$log_level" \
    --env-file "$root_dir/$env_file" \
    "${mounts[@]}" \
    --platform linux/amd64 \
    "$image"
}

if [[ "$mode" == native ]]; then
  ensure_native_tools
  run_native_super_linter
elif [[ "$mode" == container ]]; then
  run_super_linter "autofix" ".github/super-linter-autofix.env"
  run_super_linter "check" ".github/super-linter-checks.env"
elif [[ "$mode" == auto ]]; then
  if ensure_native_tools; then
    run_native_super_linter
  else
    echo "Falling back to container mode because native tools are missing." >&2
    run_super_linter "autofix" ".github/super-linter-autofix.env"
    run_super_linter "check" ".github/super-linter-checks.env"
  fi
else
  echo "Unknown mode: $mode (expected: native, container, auto)" >&2
  exit 2
fi
