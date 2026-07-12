# Spec 04: VS Code and agent formatting/linting workflow

- **Status**: proposed
- **Depends on**: 01, 02, 03
- **Size**: S

## Objective

Make VS Code extensions a reliable fast-feedback workflow without giving them a separate style
policy, and provide concise cross-language instructions for contributors and agents.

## VS Code design

Configure language-scoped ownership in `Dr.QP.code-workspace`:

| Language                    | Default formatter / diagnostics                                                  |
| --------------------------- | -------------------------------------------------------------------------------- |
| Python                      | Ruff, using repository `ruff.toml` and workspace binary where supported          |
| C/C++                       | clangd/clang-format, using root `.clang-format` and container toolchain          |
| Markdown                    | project Prettier; markdownlint diagnostics only if spec 02 adds a pinned CI gate |
| JSON/JSONC and generic YAML | project Prettier                                                                 |
| Ansible                     | Red Hat Ansible diagnostics; Super-Linter owns `ansible-lint`; never generic Prettier |
| Bash                        | shell language tooling plus ShellCheck diagnostics if an extension is adopted    |
| Dockerfile                  | Docker extension plus Hadolint diagnostics if an extension is adopted            |
| XML                         | Red Hat XML diagnostics; ament xmllint remains the package gate                  |

Choose one of two explicit save policies:

- enable `editor.formatOnSave` per formatter-owned language in the checked-in workspace; or
- leave it disabled but document the setting as a supported personal override.

Do not use a global default formatter. Ansible YAML must remain associated with the Ansible
language so it does not enter the generic YAML formatter path.

Add tasks backed only by repository entry points:

- `Dr.QP format staged files`
- `Dr.QP format changed files`
- `Dr.QP check formatting`
- `Dr.QP fast lint`
- `Dr.QP lint affected ROS packages`
- `Dr.QP full ROS lint`

Tasks should include problem matchers where stable output exists and otherwise link to the
generated log. They must not duplicate tool arguments maintained by the scripts.

## Human and agent guidance

- Add a short quality section to `CONTRIBUTING.md` with bootstrap, fix, fast-check, and full-check
  commands, plus the automatically installed pre-commit/pre-push behavior and measured performance
  expectations.
- Add a short ownership table to `AGENTS.md`; keep it policy-level.
- Create one cross-language formatting/linting skill covering command selection, ownership, and
  failure routing. Link to focused references for Python/ament, C++/ament, and Super-Linter's
  Ansible workflow rather than growing one giant troubleshooting file.
- Reconcile the existing Python skill with the new entry points. It should keep the valuable Ruff
  versus ament_flake8 explanation but stop teaching deprecated script names.
- State that editor success is fast feedback, not proof of full ament compliance.

## Extension policy

Every recommended quality extension must satisfy at least one of:

1. it uses the same repository configuration and checker as CI;
2. it is explicitly documented as advisory syntax/schema feedback.

For markdownlint and any future ShellCheck/Hadolint extensions, either pin/configure an equivalent
CLI gate or label the extension advisory. Remove duplicate recommendations and extensions that
compete for the same language's formatting provider.

## Test plan

- Open representative Python, C++, Markdown, generic YAML, and Ansible files in the devcontainer
  and run Format Document; compare the result with `scripts/format.sh --write <file>`.
- Verify an Ansible file is not offered to Prettier by default.
- Verify each workspace task works from a Git worktree and reports the same result as its script.
- Verify devcontainer setup installs the hooks once and a no-op commit does not start unrelated
  tools.
- Verify a fresh devcontainer installs all recommended extensions and project tool dependencies.
- Validate documentation commands in CI or a lightweight documentation test.
- Run the agent-file validator after adding/updating guidance and skills.

## Acceptance criteria

- [ ] Every formatter-owned language has one VS Code default formatter.
- [ ] Save-time policy is explicit and preserves user opt-out.
- [ ] Ansible never enters the generic Prettier path.
- [ ] Workspace tasks delegate to repository scripts.
- [ ] Devcontainer setup installs pre-commit and pre-push hooks idempotently.
- [ ] Contributor guidance lists fix, fast-check, and full-check commands.
- [ ] Agents have cross-language ownership guidance, with Python's ament distinction preserved.
- [ ] Recommended editor-only diagnostics are clearly marked advisory or have matching CI gates.
