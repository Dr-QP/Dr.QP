# Spec 04: VS Code and agent formatting/linting workflow

- **Status**: proposed
- **Depends on**: 01, 02, 03
- **Size**: S

## Objective

Make editor feedback and contributor/agent guidance reflect the authoritative repository owners.
Do not present an extension formatter as CI-equivalent when Super-Linter owns that file class.

## VS Code ownership

| Language                    | Editor role                                                        | Authoritative repository gate                 |
| --------------------------- | ------------------------------------------------------------------ | --------------------------------------------- |
| Python                      | Ruff formatter/diagnostics using repository configuration          | uv-backed Ruff script plus ament package lint |
| Notebook Markdown/code      | Editing feedback only; use the explicit notebook task              | notebook scripts, then Super-Linter Prettier  |
| C/C++                       | clangd formatting/diagnostics are fast feedback                     | Super-Linter clang-format plus ament checks   |
| Markdown, YAML, JSON/JSONC  | Prettier extension is advisory unless parity is proven              | Super-Linter Prettier                         |
| Ansible                     | Red Hat Ansible and local ansible-lint diagnostics are advisory     | Super-Linter ansible-lint                     |
| Bash                        | Shell tooling and optional ShellCheck diagnostics                   | pre-commit and Super-Linter ShellCheck        |
| Dockerfile                  | Docker tooling; Hadolint extension is advisory unless parity-tested | Super-Linter Hadolint                         |
| XML                         | Red Hat XML diagnostics                                             | ament xmllint                                 |

Python may use a checked-in default formatter because Ruff is the established native owner.
For C++, Prettier-owned files, and Ansible, either prove extension output/version parity with the
pinned Super-Linter image or label Format Document as preview feedback and direct users to the
repository fix/check tasks before push.

Do not use one global default formatter. Keep Ansible YAML associated with the Ansible language so
generic YAML formatting cannot rewrite it.

Choose and document one save policy:

- enable save-time formatting only for surfaces proven equivalent to their authoritative owner; or
- keep save-time formatting disabled and document supported personal overrides.

## Workspace tasks

Add tasks backed only by repository entry points:

- `Dr.QP format changed files`
- `Dr.QP format all files`
- `Dr.QP check formatting and fast lint`
- `Dr.QP run Super-Linter`
- `Dr.QP lint affected ROS packages`
- `Dr.QP full quality gate`
- retain explicit `Dr.QP format notebooks` and `Dr.QP sync notebooks` operations

Tasks must delegate to scripts without copying tool flags. Add problem matchers only where output is
stable; otherwise link to `log/super-linter-summary.md`, detailed Super-Linter outputs, or ROS test
logs.

## Human and agent guidance

- Add a short quality section to `CONTRIBUTING.md` with four common paths: ordinary Python,
  notebooks, heterogeneous/C++/Ansible/docs changes, and the full ROS gate.
- Explain that pre-commit is fast changed-file feedback and list the actual pre-push behavior chosen
  by spec 02.
- Add a policy-level ownership table to `AGENTS.md`.
- Keep the cross-language formatting/linting skill centered on command selection and failure
  routing. Link to focused Python/ament and Super-Linter result guidance.
- Preserve the existing Ruff-versus-ament_flake8 explanation.
- State explicitly that editor success does not prove Super-Linter or ament compliance.
- State explicitly that `python-reformat.sh` does not format/synchronize notebooks.

## Extension policy

Every recommended quality extension must satisfy one of:

1. it is configured and tested against the same repository owner/version as CI; or
2. it is labeled advisory syntax/schema/preview feedback.

For markdownlint and future ShellCheck/Hadolint extensions, add a pinned/configured matching gate or
label them advisory. Remove duplicate recommendations and extensions that compete for the same
language's editor formatter slot.

## Test plan

- Open representative Python, C++, notebook Markdown, generic YAML, and Ansible files in a fresh
  devcontainer and record the offered formatter/diagnostics.
- Compare Python Format Document output with the repository Ruff command.
- For C++/Prettier/Ansible, either prove representative parity with Super-Linter or verify guidance
  marks editor output advisory.
- Verify Ansible files never enter the generic YAML Prettier path.
- Verify every workspace task works from a Git worktree and delegates to the expected script.
- Verify notebook format and sync tasks remain distinct.
- Validate documentation commands and run the agent-file validator after guidance changes.

## Acceptance criteria

- [ ] Editor settings distinguish authoritative formatting from advisory feedback.
- [ ] Python has one Ruff default formatter and an explicit save policy.
- [ ] Ansible never enters the generic Prettier path.
- [ ] Workspace tasks delegate to repository scripts and expose focused notebook operations.
- [ ] Contributor guidance lists fix, fast-check, Super-Linter, and full ROS commands.
- [ ] Guidance matches the configured pre-commit/pre-push stages.
- [ ] Agents preserve Super-Linter ownership and the Ruff-versus-ament distinction.
- [ ] Recommended editor-only diagnostics are marked advisory or have matching parity evidence.
