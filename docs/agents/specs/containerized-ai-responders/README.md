# Containerized AI responders

This spike specifies a container-backed Codex responder alongside the existing
Claude responder. It is deliberately a workflow integration rather than the
hosted Codex code-review integration: every enabled responder must execute in
the repository's ROS Jazzy container and therefore have the same ability to
inspect code and run verification commands.

## Decision

Use `openai/codex-action@v1` in a new `codex-review.yml` workflow. It runs
`codex exec` in the job container; it is not the GitHub-hosted `@codex review`
feature, which delegates work to Codex Cloud and cannot use this repository's
container image.

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

## Current state

- `.github/workflows/claude-review.yml` already responds in
  `ghcr.io/dr-qp/jazzy-ros-desktop:edge`, resolves the actual pull-request head
  SHA for comment events, and limits write-token delivery when its own workflow
  file changes.
- The repository's `pr-review` skill explicitly permits `@codex review` and
  tells Codex to publish a real GitHub pull-request review.
- `require-ai-review.yml` already accepts GitHub Actions reviews as an AI review
  source. A review posted with `GITHUB_TOKEN` will appear as `github-actions[bot]`.
- The documented `openai/codex-action@v1` installs Codex and runs `codex exec`.
  Its `sandbox: danger-full-access` mode is appropriate only for this isolated,
  trusted container job.

## Issue register

### P0

1. **Untrusted event content must not receive secrets or write authority.**
   PR descriptions and comments are prompt-injection inputs; fork code is also
   untrusted. The workflow must use an authorization gate before the Codex step,
   pass credentials only to that step, and use the same workflow-file-change
   restriction as Claude.
2. **A review must be published as a GitHub pull-request review.** A Codex final
   message is not, by itself, a review. The review prompt and available tooling
   must lead to the `pr-review` skill's proper review flow, including an approval
   when there are no findings.

### P1

3. **Responder behavior can drift.** Claude and Codex must share the event
   matrix, checkout-ref logic, authorization rules, prompts, token scope, and
   artifact/debug behavior. Keep workflow-specific action wiring small and
   compare the common behavior in tests.
4. **`scripts/post-review.sh` is referenced by the review skill but is absent.**
   Before relying on its fallback path, either add and test that helper or make
   the Codex workflow provide known working review-posting tools.
5. **`AI_RESPONDERS` is a free-form string.** Validate it early and fail clearly
   on unsupported values rather than silently disabling protection or creating
   unexpected API spend.

### P2

6. **Automated dual reviews increase API cost and feedback duplication.** Start
   with mention-triggered Codex runs; enable automatic Codex reviews only after
   measuring usefulness and cost.
7. **Review identity is ambiguous.** Both action-based responders normally post
   as `github-actions[bot]`. Retain an unambiguous review-body marker and do not
   change the existing AI-review gate without proving its semantics.

## Implementation order

| Order | Spec | Result |
| ----- | ---- | ------ |
| 1 | [01 — Containerized Codex responder and toggle](01-containerized-codex-responder-and-toggle.md) | A secure, selectively enabled Codex responder with parity tests |

## Non-goals

- Enabling Codex Cloud's built-in automatic review or cloud task execution.
- Replacing Claude, changing the current `require-ai-review` policy, or
  granting agents permission to push branches.
- Allowing untrusted fork pull requests to run a credentialed responder.
