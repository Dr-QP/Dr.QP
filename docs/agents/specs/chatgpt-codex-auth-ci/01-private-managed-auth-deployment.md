# Spec 01: Private-only managed-auth deployment

- **Status**: proposed, blocked by repository visibility
- **Depends on**: the target repository and every workflow using this account
  credential being private and trusted; a provisioned cloud secret manager or
  Vault; a seeded file-backed ChatGPT Codex credential
- **Size**: M

## Objective

For a private, trusted GitHub-hosted runner workflow, replace
`openai/codex-action` with the Codex CLI and round-trip the **refreshed**
`auth.json` through an OIDC-protected external secret manager. Do not apply
this spec to `Dr-QP/Dr.QP` while it remains public.

## Preconditions

1. The target repository is private and only trusted maintainers can alter the
   workflow, prompts, and checked-out source that reaches the agent.
2. On a trusted interactive machine, set
   `cli_auth_credentials_store = "file"`, run `codex login`, and inspect the
   resulting `auth.json`. It must report `auth_mode: "chatgpt"` and contain a
   non-empty refresh token.
3. Seed that file once in a cloud secret manager or Vault. Do not seed it as an
   Actions cache, artifact, repository file, or ordinary workflow output.
4. Configure GitHub OIDC trust for the exact repository, environment,
   workflow, and protected ref. Grant the resulting cloud identity only read
   and replace/update access to this one secret.
5. Choose one provider implementation (AWS Secrets Manager, Google Secret
   Manager, Azure Key Vault, or Vault) and document its exact CLI/action
   version and IAM policy in the private repository.

## Workflow design

### Serialization

Every job that uses this credential must share an explicit job-level
concurrency group. Do not derive it from an issue, pull request, or event ID.
Do not cancel an in-progress run, because it may be between a successful token
refresh and its write-back step.

```yaml
concurrency:
  group: codex-chatgpt-auth-account-1
  cancel-in-progress: false
```

GitHub Actions permits at most one running job for a concurrency group. If the
workflow must process every queued request rather than allowing GitHub's normal
pending-run replacement behavior, introduce an application-level queue before
the credentialed job.

### Least privilege

Keep existing repository permissions minimal and add only the OIDC grant:

```yaml
permissions:
  contents: read
  id-token: write
```

Grant issue/pull-request write permissions only to a separate reply/publish
job where possible. The credentialed Codex job must not receive a GitHub App
or PAT with `Secrets: write`; the cloud secret manager is the mutable store.

### Restore, execute, persist

Use one `CODEX_HOME` in all three steps. In a container job, use the same
container home directory in all shell steps, for example `$HOME/.codex`; do
not assume a hosted-runner host path is meaningful inside the container.

The following is provider-neutral pseudocode. Replace `secretctl` with the
chosen provider's authenticated CLI or pinned official action, without
printing secret contents.

```yaml
- name: Restore managed Codex auth
  id: restore_codex_auth
  shell: bash
  run: |
    set -euo pipefail
    export CODEX_HOME="$HOME/.codex"
    mkdir -p "$CODEX_HOME"
    chmod 700 "$CODEX_HOME"
    secretctl read codex-auth-json > "$CODEX_HOME/auth.json"
    chmod 600 "$CODEX_HOME/auth.json"
    jq -e '
      .auth_mode == "chatgpt" and
      ((.tokens.refresh_token // "") != "")
    ' "$CODEX_HOME/auth.json" >/dev/null

- name: Run Codex CLI
  id: run_codex
  shell: bash
  run: |
    set -euo pipefail
    export CODEX_HOME="$HOME/.codex"
    codex exec --json "$CODEX_PROMPT" > codex-output.json

- name: Persist refreshed managed Codex auth
  if: always() && steps.restore_codex_auth.outcome == 'success'
  shell: bash
  run: |
    set -euo pipefail
    export CODEX_HOME="$HOME/.codex"
    test -s "$CODEX_HOME/auth.json"
    jq -e '
      .auth_mode == "chatgpt" and
      ((.tokens.access_token // "") != "") and
      ((.tokens.refresh_token // "") != "") and
      (.last_refresh != null)
    ' "$CODEX_HOME/auth.json" >/dev/null
    secretctl write codex-auth-json < "$CODEX_HOME/auth.json"
```

`secretctl write` must receive standard input from the current on-disk file.
It must not receive `${{ secrets.CODEX_AUTH_JSON }}`, a shell variable holding
the initial seed, or any serialized workflow output. The provider integration
should use its conditional-update/version mechanism when available, so an
unexpected concurrent writer fails rather than silently overwriting a newer
version.

Install a pinned Codex CLI before the execution step through the deployment's
approved package installation route. Do not retain `openai/codex-action` in
the same job: that action configures an API-key proxy and does not implement
ChatGPT-managed file authentication.

## Test plan

- In an isolated private test repository, restore a non-secret fixture with
  `auth_mode: "chatgpt"` and verify validation accepts only the required shape.
- Verify a missing file, wrong `auth_mode`, missing refresh token, or malformed
  JSON fails before `codex exec` is called.
- Stub `codex` to rewrite `auth.json`; verify the persistence command reads the
  rewritten file rather than the restore input.
- Stub a failing `codex` process that rewrites `auth.json`; verify the
  `always()` persistence step executes and stores the rewritten file.
- Verify serialization by triggering two jobs with the same concurrency group;
  their Codex executions and writes must not overlap.
- Verify no workflow command logs the file, a token field, or a secret-manager
  response containing credential material.
- Verify the credential does not appear in Actions cache paths, artifacts,
  repository files, or job outputs.
- Test the OIDC trust policy rejects a different repository, branch,
  environment, and workflow.

## Acceptance criteria

- [ ] The target repository and triggering workflow are private and trusted.
- [ ] Codex is invoked directly through the CLI, not `openai/codex-action`.
- [ ] The secret manager is authenticated using job-scoped GitHub OIDC.
- [ ] Restore, Codex execution, and `always()` write-back use the same
      `CODEX_HOME/auth.json` file.
- [ ] The write-back stores the file Codex left on disk, with structural
      validation before update.
- [ ] All consumers of the credential share a stable concurrency group.
- [ ] Cache, artifacts, logs, repository files, workflow outputs, and GitHub
      secret mutation are absent from the credential path.
- [ ] The provider trust policy and secret permissions are least-privilege and
      documented in the private deployment repository.
