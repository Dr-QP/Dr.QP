#!/usr/bin/env bash

set -euo pipefail

url=""
format="fields"
include_log=0
grep_failures=0

usage() {
  cat <<'EOF'
Parse a GitHub Actions run or job URL into fields or a gh command.

Usage:
  parse-actions-url.sh --url <github-actions-url> [--format fields|command] [--log] [--grep-failures]

Options:
  --url <url>          GitHub Actions run or job URL to parse.
  --format <mode>      Output mode: fields or command. Default: fields.
  --log                Include --log when generating a gh command.
  --grep-failures      Append the standard failure grep filter in command mode.
  -h, --help           Show this help text.

Exit codes:
  0  Success
  2  Invalid arguments or unsupported URL

Examples:
  parse-actions-url.sh --url 'https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222'
  parse-actions-url.sh --url 'https://github.com/Dr-QP/Dr.QP/actions/runs/26806349222/job/79025184711?pr=366' --format command --log
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --url)
      [[ $# -ge 2 ]] || { echo "Missing value for --url" >&2; exit 2; }
      url="$2"
      shift 2
      ;;
    --format)
      [[ $# -ge 2 ]] || { echo "Missing value for --format" >&2; exit 2; }
      format="$2"
      shift 2
      ;;
    --log)
      include_log=1
      shift
      ;;
    --grep-failures)
      grep_failures=1
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

if [[ -z "$url" ]]; then
  echo "Missing required --url argument" >&2
  usage >&2
  exit 2
fi

if [[ "$format" != "fields" && "$format" != "command" ]]; then
  echo "Unsupported --format value: $format" >&2
  exit 2
fi

if [[ "$grep_failures" -eq 1 && "$format" != "command" ]]; then
  echo "--grep-failures requires --format command" >&2
  exit 2
fi

if [[ "$url" =~ ^https://github\.com/([^/]+)/([^/]+)/actions/runs/([0-9]+)(/attempts/[0-9]+)?(/job/([0-9]+))?([?#].*)?$ ]]; then
  repo="${BASH_REMATCH[1]}/${BASH_REMATCH[2]}"
  run_id="${BASH_REMATCH[3]}"
  job_id="${BASH_REMATCH[6]:-}"
else
  echo "invalid GitHub Actions URL: $url" >&2
  exit 2
fi

if [[ "$format" == "fields" ]]; then
  printf 'REPO=%q\n' "$repo"
  printf 'RUN_ID=%q\n' "$run_id"
  if [[ -n "$job_id" ]]; then
    printf 'JOB_ID=%q\n' "$job_id"
  fi
  exit 0
fi

command=(gh run view "$run_id" --repo "$repo")
if [[ -n "$job_id" ]]; then
  command+=(--job "$job_id")
fi
if [[ "$include_log" -eq 1 ]]; then
  command+=(--log)
fi

printf -v rendered '%q ' "${command[@]}"
rendered="${rendered% }"

if [[ "$grep_failures" -eq 1 ]]; then
  printf '%s | grep -nE %q\n' "$rendered" 'FAILED|FAILURES|AssertionError|ERROR:|Segmentation fault|test_'
else
  printf '%s\n' "$rendered"
fi