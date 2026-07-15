#!/usr/bin/env bash
# Submit one pull-request review with every validated inline finding.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/post-review.sh --pr NUMBER --event APPROVE|COMMENT \
  --summary-file PATH [--comments-file PATH] [--repo OWNER/REPOSITORY]

The comments file must contain a JSON array accepted by GitHub's pull-request
review API. The script makes exactly one review creation request.
EOF
}

pr_number=''
review_event=''
summary_file=''
comments_file=''
repository=''

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pr)
      pr_number=${2:?missing pull request number}
      shift 2
      ;;
    --event)
      review_event=${2:?missing review event}
      shift 2
      ;;
    --summary-file)
      summary_file=${2:?missing summary file path}
      shift 2
      ;;
    --comments-file)
      comments_file=${2:?missing comments file path}
      shift 2
      ;;
    --repo)
      repository=${2:?missing repository}
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

if [[ ! "$pr_number" =~ ^[1-9][0-9]*$ ]]; then
  echo '--pr must be a positive pull request number.' >&2
  exit 2
fi

if [[ "$review_event" != 'APPROVE' && "$review_event" != 'COMMENT' ]]; then
  echo '--event must be APPROVE or COMMENT.' >&2
  exit 2
fi

if [[ ! -f "$summary_file" ]]; then
  echo '--summary-file must name an existing file.' >&2
  exit 2
fi

if [[ -n "$comments_file" && ! -f "$comments_file" ]]; then
  echo '--comments-file must name an existing file.' >&2
  exit 2
fi

if [[ -z "$repository" ]]; then
  repository=$(gh repo view --json nameWithOwner --jq .nameWithOwner)
fi

if [[ -n "$comments_file" ]]; then
  comments=$(jq -ce 'if type == "array" then . else error("comments must be an array") end' "$comments_file")
else
  comments='[]'
fi

if [[ "$review_event" == 'APPROVE' && "$comments" != '[]' ]]; then
  echo 'An APPROVE review cannot include inline findings.' >&2
  exit 2
fi

payload=$(jq -cn \
  --rawfile body "$summary_file" \
  --arg event "$review_event" \
  --argjson comments "$comments" \
  '{body: $body, event: $event, comments: $comments}')

gh api --method POST "repos/$repository/pulls/${pr_number}/reviews" --input - <<<"$payload"
