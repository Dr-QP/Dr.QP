# Supported Read the Docs Build Configuration

## Objective

Install Ubuntu's `ffmpeg` package before executing the Sphinx documentation
build on Read the Docs.

## Implementation

1. Keep `build.os: ubuntu-24.04`, the existing Python and Node.js selections,
   and `build.apt_packages: [ffmpeg]`.
2. Remove `build.commands`; it disables the APT system-dependency phase.
3. Move the existing frozen `uv` install and Sphinx commands to `build.jobs`.
   The structured jobs preserve `build.apt_packages` and retain the existing
   command semantics.
4. Add `ffmpeg -version` as a `pre_build` job to prove the system dependency is
   on `PATH` before Sphinx executes notebook rendering.

## Acceptance criteria

- A Read the Docs build installs `ffmpeg` in the system-dependency stage before
  its Python-environment and Sphinx stages.
- The `pre_build` output reports the installed `ffmpeg` version.
- Notebook animation rendering can locate `ffmpeg` on `PATH`.
- The rendered site is produced in `$READTHEDOCS_OUTPUT/html` and the build
  succeeds.
- No package is installed through `sudo`, a PPA, or a custom repository.

## Validation

Trigger a new Read the Docs build after merging the configuration. In its log,
verify the APT system-dependency stage lists `ffmpeg`, followed by the existing
frozen `uv` install, the `ffmpeg -version` check, and Sphinx HTML build.
