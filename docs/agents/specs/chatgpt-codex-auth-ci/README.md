# ChatGPT-managed Codex authentication in CI

## Decision

Do **not** add `CODEX_AUTH_JSON` or ChatGPT-managed Codex authentication to
this repository's GitHub Actions workflows. The repository is public and its
AI responder executes Codex against event-controlled repository content. The
official OpenAI CI/CD guidance limits this authentication method to trusted,
private automation and explicitly excludes public or open-source repositories.

Continue to use an OpenAI API key for the public `ai-responder.yml` workflow,
or disable its Codex responder. A ChatGPT subscription credential must never
be placed in an Actions secret, cache, artifact, workspace file, or log in this
repository.

This program documents the conditional design for a future **private,
trusted** repository or isolated private automation repository. It is not
authorization to implement that design here.

## Raw findings

### Authentication mechanisms are incompatible

- `openai/codex-action@v1.11` requires its `openai-api-key` input, starts a
  Responses API proxy only when that input is populated, and configures Codex
  to use that proxy. It does not authenticate from `~/.codex/auth.json`.
- The Codex CLI can instead use file-backed ChatGPT-managed credentials from
  `~/.codex/auth.json` when `auth_mode` is `chatgpt` and the token bundle has a
  refresh token.
- A normal `codex exec` performs built-in refresh when needed and writes the
  refreshed token bundle and `last_refresh` back to the same file. CI must
  therefore persist the post-run file, never reseed each run with the original
  value.

### GitHub-hosted runners require mutable external state

- GitHub-hosted runners are ephemeral. A direct CLI workflow needs to restore
  `auth.json` before Codex runs and write that same on-disk file to durable
  secure storage afterwards.
- GitHub Actions cache is not suitable: GitHub warns not to store access tokens
  or credentials in cache paths, because repository contributors can retrieve
  cached contents through pull-request workflows.
- GitHub Actions repository/environment secrets are input storage, not the
  preferred refresh store. Updating one requires a separate GitHub App or
  fine-grained token with `Secrets: write`, which creates another high-value
  secret in the agent job.
- The preferred durable store is a cloud secret manager or Vault. GitHub OIDC
  gives the job short-lived credentials to read and replace exactly one secret;
  the cloud trust policy must restrict the repository, workflow, ref, and
  environment.

### The current workflow is unsuitable

`.github/workflows/ai-responder.yml` is triggered by issues, comments, pull
request review comments, pull requests, and review submissions. It checks out
same-repository pull-request heads, runs in a container with
`sandbox: danger-full-access`, and grants GitHub write permissions for replies
and reviews. It authorizes requesters and rejects forks, but that does not make
a public repository a private trusted automation boundary.

The workflow's existing top-level concurrency key is event-specific. It does
not serialize all runs that would share one ChatGPT credential file. Any
private deployment of this design needs a stable job-level concurrency group
shared by every job that uses that credential.

## Issue register

### P0 — public repository credential exposure

1. A ChatGPT-managed `auth.json` is equivalent to an account password and is
   prohibited by the official flow for public/open-source repositories. Adding
   it to the current responder would expose a long-lived, refreshable account
   credential to a public, agentic workflow.

2. Switching only from `openai/codex-action` to `codex exec` does not solve
   persistence. An ephemeral runner discards the refreshed file unless the
   workflow writes the updated file to mutable secure storage after every run.

### P1 — stale or corrupted credential state

3. Reseeding `auth.json` from the original GitHub secret on every run erases
   the refreshed token bundle. Subsequent runs can eventually fail even though
   Codex successfully refreshed a previous run.

4. Concurrent jobs can both refresh one credential copy and then write back in
   an arbitrary order. The later writer can overwrite a newer token bundle.

5. A cancelled or failed Codex step may still have refreshed `auth.json`.
   Persistence must run with `always()` after a successful restore, while
   rejecting missing or malformed files.

### P2 — unsuitable persistence shortcuts

6. Actions cache and artifacts must not carry the credential. Cache access and
   artifact retention are inappropriate for an account refresh token.

7. A GitHub App or PAT able to mutate repository secrets is possible but adds a
   second privileged credential to the same agent job. Prefer OIDC to an
   external secret manager.

## Conditional implementation order

| Order | Spec | Result |
| ----- | ---- | ------ |
| 1 | [01 — Private-only managed-auth deployment](01-private-managed-auth-deployment.md) | A private, serialized direct-CLI job round-trips the refreshed `auth.json` through an OIDC-protected secret manager. |

## Scope boundaries

- Moving this public repository to private visibility.
- Creating or renewing a ChatGPT session with `codex login` from CI.
- Calling an OAuth refresh endpoint directly; Codex owns refresh.
- Persisting credential material through the repository, Actions cache,
  artifacts, logs, or workflow outputs.
- Updating a GitHub Actions secret from the agent job with a GitHub App or PAT.
- Replacing the current public responder's API-key authentication.

## Source material

- OpenAI, [Maintain Codex account auth in CI/CD (advanced)](https://learn.chatgpt.com/docs/auth/ci-cd-auth).
- `openai/codex-action`, [v1.11 action definition](https://github.com/openai/codex-action/blob/v1.11/action.yml) and [README](https://github.com/openai/codex-action/tree/v1.11/).
- GitHub, [OpenID Connect reference](https://docs.github.com/en/actions/reference/security/oidc), [dependency-cache security guidance](https://docs.github.com/en/actions/reference/workflows-and-actions/dependency-caching), and [Actions Secrets API](https://docs.github.com/en/rest/actions/secrets).
