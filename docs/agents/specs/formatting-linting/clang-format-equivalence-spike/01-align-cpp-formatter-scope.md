# Spec 01: Align C++ formatter scope before enabling Super-Linter clang-format

- **Status**: proposed
- **Depends on**: [formatting-linting spec 01](../01-ownership-and-entry-points.md)
- **Size**: S

## Objective

Ensure one explicit C/C++ formatter scope is used by the repository entry point and any
Super-Linter clang-format check or autofix job.

## Design

Choose and document one vendor policy:

1. Exclude `packages/runtime/drqp_rapidjson/include/**` from every repository formatter; or
2. Remove its `AMENT_IGNORE` marker and deliberately accept a one-time, upstream-vendor formatting
   patch.

The recommended policy is option 1: retain imported RapidJSON verbatim. Encode the exclusion once
in the shared C++ scope resolver required by formatting-linting spec 01, then make both the native
clang-format command and Super-Linter consume that resolver or its generated include/exclude list.

If Super-Linter clang-format is enabled, make all of the following explicit:

- `VALIDATE_CLANG_FORMAT=true` for checks and `FIX_CLANG_FORMAT=true` only for the chosen autofix
  workflow;
- configuration lookup through `LINTER_RULES_PATH=.` or a maintained
  `.github/linters/.clang-format` copy/symlink;
- the Super-Linter image tag and the expected local/devcontainer clang-format major version;
- check-only mode for normal CI gates, with only the designated formatter owner allowed to write.

## Test plan

- In disposable copies, run each formatter over the full C++ scope and compare trees with
  `git diff --no-index`.
- Confirm the RapidJSON subtree is excluded by both tools under the selected policy.
- Include a deliberately unformatted non-vendor C++ fixture and verify both tools make the same
  byte-for-byte change.
- Record both clang-format versions in test output; fail or skip the parity assertion when they do
  not match unless a version-drift compatibility test explicitly passes.
- Verify the normal Super-Linter environment still leaves C++ untouched unless C++ ownership is
  intentionally moved to that workflow.

## Acceptance criteria

- [ ] C++ formatter ownership and excluded paths are defined once.
- [ ] Root `.clang-format` is the effective configuration for every enabled clang-format surface.
- [ ] Ament and Super-Linter produce identical output for the selected C++ scope in a pinned test.
- [ ] Imported vendor code is not reformatted accidentally.
- [ ] CI has one designated C++ formatter writer.
