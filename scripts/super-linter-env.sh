#!/usr/bin/env bash

set -euo pipefail

autofix="${SUPER_LINTER_AUTOFIX:-false}"
enable_all_checks=false

usage()
{
  cat <<'EOF'
Usage: scripts/super-linter-env.sh [--autofix]

Generate env file for Super-Linter autofix pass and check pass

Options:
  --autofix            Run autofixes.
  --enable_all_checks  Enable all checks, including those that don't support autofix.
EOF
}

while (($#)); do
  case "$1" in
    --autofix)
      autofix=true
      shift
      ;;
    --enable_all_checks)
      enable_all_checks=true
      shift
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

echo FILTER_REGEX_EXCLUDE='(^|/)(build|install|log|\.venv|packages/vendor|packages/runtime/drqp_rapidjson/include|\.git)/'

if [[ "$autofix" == "false" || "$enable_all_checks" == "true" ]]; then
  # Check only flags for linters that don't support autofix.
  echo VALIDATE_BASH=true
  echo VALIDATE_DOCKERFILE_HADOLINT=true
  echo VALIDATE_GITHUB_ACTIONS=true
  echo VALIDATE_GITLEAKS=true
fi
echo VALIDATE_MARKDOWN_PRETTIER=true
echo VALIDATE_YAML_PRETTIER=true
echo VALIDATE_JSON_PRETTIER=true
echo VALIDATE_JSONC_PRETTIER=true
echo VALIDATE_GITHUB_ACTIONS_ZIZMOR=true
echo VALIDATE_CLANG_FORMAT=true
echo VALIDATE_ANSIBLE=true
echo ANSIBLE_DIRECTORY=docker/ros/ansible

echo FIX_MARKDOWN_PRETTIER=${autofix}
echo FIX_YAML_PRETTIER=${autofix}
echo FIX_JSON_PRETTIER=${autofix}
echo FIX_JSONC_PRETTIER=${autofix}
echo FIX_GITHUB_ACTIONS_ZIZMOR=${autofix}
echo FIX_CLANG_FORMAT=${autofix}
echo FIX_ANSIBLE=${autofix}

echo SAVE_SUPER_LINTER_SUMMARY=true
