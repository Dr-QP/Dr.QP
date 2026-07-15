# Containerized AI responder specifications

Implement a container-backed Codex responder alongside the existing Claude
responder. Every enabled responder executes in the repository's ROS Jazzy
container and can inspect code and run the same verification commands.

Use `openai/codex-action` in `.github/workflows/codex-review.yml`. The action
runs `codex exec` in the job container. Do not use GitHub-hosted `@codex
review`, because that feature delegates work to Codex Cloud rather than this
repository's container image.

Both responder workflows are controlled by the repository configuration
variable `AI_RESPONDERS`, exposed as a workflow environment value. Its allowed
values are a lowercase, comma-separated set with no whitespace:

| `AI_RESPONDERS` value | Enabled responders |
| -------------------- | ------------------ |
| unset or `claude`    | Claude only (backward-compatible default) |
| `codex`              | Codex only |
| `claude,codex`       | Claude and Codex |
| `none`               | Neither |

The variable is an ordinary GitHub Actions repository or environment variable,
not a secret. Credentials remain separate secrets. The implementation must use
exact-token matching, so `notclaude` must never enable Claude.

## Implementation constraints

- Preserve the execution model in `.github/workflows/claude-review.yml`:
  `ghcr.io/dr-qp/jazzy-ros-desktop:edge`, PR-head checkout for comment events,
  and withheld agent write tokens when protected workflow files change.
- Use the existing `pr-review` skill for `@codex review`; Codex must publish a
  GitHub pull-request review, including `APPROVE` when it finds no issues.
- Keep the existing `require-ai-review.yml` policy. Action-posted reviews appear
  as `github-actions[bot]`; add an agent marker to the review summary so the
  workflow output remains auditable.
- Treat all PR text, issue text, comments, diffs, and checked-out code as
  untrusted input. Authorization occurs before the agent action receives an API
  key or a GitHub write token.
- Validate `AI_RESPONDERS` before checkout. Invalid values fail the job rather
  than disabling an expected reviewer or creating an unexpected API call.
- Add and test `scripts/post-review.sh`, the fallback review publisher required
  by the `pr-review` skill. It must use one API call for the review and its
  inline comments.

## Implementation order

| Order | Spec | Result |
| ----- | ---- | ------ |
| 1 | 01 — Containerized Codex responder and toggle | Implemented: secure, selectively enabled Codex responder with parity tests |

## Scope boundaries

- Enabling Codex Cloud's built-in automatic review or cloud task execution.
- Replacing Claude, changing the current `require-ai-review` policy, or
  granting agents permission to push branches.
- Allowing untrusted fork pull requests to run a credentialed responder.
