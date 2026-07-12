# clang-format equivalence spike

- **Status**: complete
- **Run date**: 2026-07-12
- **Scope**: `packages/` at commit `1a291b5c549e9a9486f8eeb028408a18968923c9`
- **Question**: Does `ament_clang_format --config .clang-format --reformat packages/`
  produce the same result as Super-Linter's clang-format autofix with that configuration?

## Conclusion

The original directory-level operations did not produce identical output. After adding the
RapidJSON exclusion to both checked-in Super-Linter environments, the repeated comparison
produced byte-identical `packages/` trees.

| Run                    | Formatter             | clang-format version | Files changed | Result                        |
| ---------------------- | --------------------- | -------------------- | ------------: | ----------------------------- |
| Before scope alignment | ament                 | 20.1.8               |             0 | Passed                        |
| Before scope alignment | Super-Linter `v8.5.0` | 21.1.2               |            38 | Reformatted RapidJSON headers |
| After scope alignment  | ament                 | 20.1.8               |             0 | Passed                        |
| After scope alignment  | Super-Linter `v8.5.0` | 21.1.2               |             0 | Passed                        |

The initial Super-Linter patch contained 17,535 insertions and 12,975 deletions in 38 RapidJSON
headers. `packages/runtime/drqp_rapidjson/include/AMENT_IGNORE` makes ament skip that tree. The
new Super-Linter exclusion matches that policy; `git diff --no-index --quiet` between the two
post-change `packages/` trees exited 0.

## Raw findings

1. `ament_clang_format` is a wrapper, not a separate formatter. It finds the first
   `clang-format` on `PATH`, converts the supplied YAML configuration to an inline `-style`,
   checks extensions `c`, `cc`, `cpp`, `cxx`, `h`, `hh`, `hpp`, and `hxx`, and skips directories
   containing `AMENT_IGNORE`.
2. The ROS environment resolved `clang-format` 20.1.8. Super-Linter `v8.5.0` bundles
   clang-format 21.1.2, so matching configuration alone does not guarantee future parity.
3. Super-Linter's default configuration directory is `.github/linters`; it does not automatically
   consume the root `.clang-format`. The comparison therefore used `LINTER_RULES_PATH=.` to make
   it consume the exact root configuration.
4. The checked-in Super-Linter autofix environment enables Markdown, YAML, JSON/JSONC, and
   Zizmor only. It currently sets neither `VALIDATE_CLANG_FORMAT=true` nor
   `FIX_CLANG_FORMAT=true`; therefore, the CI Super-Linter job does not currently format C/C++.
5. Both checked-in Super-Linter environments now exclude
   `packages/runtime/drqp_rapidjson/include`, matching ament's `AMENT_IGNORE` boundary. The
   post-change rerun changed no C/C++ files under `packages/`.

## Reproduction

Use fresh clones or another disposable copy; both tools modify files in place.

```bash
scripts/with-ros-env.sh \
  ament_clang_format --config .clang-format --reformat packages/

docker run --rm --platform linux/amd64 \
  -e RUN_LOCAL=true \
  -e DEFAULT_BRANCH=<existing-branch> \
  -e VALIDATE_ALL_CODEBASE=true \
  -e VALIDATE_CLANG_FORMAT=true \
  -e FIX_CLANG_FORMAT=true \
  -e LINTER_RULES_PATH=. \
  -e 'FILTER_REGEX_EXCLUDE=(^|/)(build|install|log|\\.venv|packages/vendor|\\.git)/' \
  -v "$PWD:/tmp/lint" \
  ghcr.io/super-linter/super-linter:v8.5.0
```

Then compare only the two `packages/` trees with `git diff --no-index`. The current exclusion
makes the tested trees identical, but the differing clang-format versions remain a future parity
risk.

## Issue register

### P1

1. **The root C++ configuration is not automatically shared.** Super-Linter defaults to
   `.github/linters/.clang-format`; the root file must be copied/symlinked there or selected with
   `LINTER_RULES_PATH=.`.
2. **Tool versions drift.** The tested paths use clang-format 20.1.8 and 21.1.2 respectively.
   Today their outputs match on the common 40-file set, but this is not a version-parity contract.

### P2

3. **C++ Super-Linter is inactive.** Current CI cannot serve as a C++ formatter comparison until
   its `VALIDATE_CLANG_FORMAT` and `FIX_CLANG_FORMAT` settings are explicitly enabled.

## Follow-up

See [01-align-cpp-formatter-scope.md](01-align-cpp-formatter-scope.md) for the implementation
choice needed before adding Super-Linter clang-format to CI.
