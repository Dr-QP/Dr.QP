# Read the Docs APT Package Investigation

## Finding

`ffmpeg` was not installed because `.readthedocs.yaml` combined
`build.apt_packages` with `build.commands`. This is an explicitly unsupported
Read the Docs combination, not an Ubuntu package availability problem.

Build 33554737 succeeded, but its log shows the configuration followed
immediately by `asdf global nodejs` and `asdf global python`; it has no system
dependency installation step. The build therefore never attempted to install
`ffmpeg`.

## Sources

- [Read the Docs configuration reference: `build.apt_packages`](https://docs.readthedocs.com/platform/stable/config-file/v2.html#build-apt-packages)
  says the setting installs APT packages, but cannot be used with
  `build.commands`.
- [Read the Docs configuration reference: `build.commands`](https://docs.readthedocs.com/platform/stable/config-file/v2.html#build-commands)
  recommends `build.jobs`, which supports `build.apt_packages`.
- [Read the Docs issue #9599](https://github.com/readthedocs/readthedocs.org/issues/9599)
  records the same limitation. It is closed because `build.jobs` was introduced
  as the supported replacement.
- [Read the Docs issue #11551](https://github.com/readthedocs/readthedocs.org/issues/11551)
  tracks that replacement and is closed as implemented.
- [Ubuntu package search: `ffmpeg`](https://packages.ubuntu.com/ffmpeg)
  lists the package for Ubuntu 24.04 (Noble), so no alternate package source is
  necessary.

## Issues

| Priority | Issue | Resolution |
| --- | --- | --- |
| P0 | `build.commands` silently bypasses `apt_packages` | Replace it with structured `build.jobs`. |
| P1 | The next hosted build must prove the system dependency phase runs | Confirm the build log contains the APT installation and `ffmpeg -version` succeeds. |

## Resolution

`.readthedocs.yaml` now uses Read the Docs' structured build jobs. They preserve
`build.apt_packages` while retaining the previous frozen `docs` dependency group
and Sphinx command verbatim. A `pre_build` `ffmpeg -version` check makes a
missing system dependency fail visibly rather than only surfacing indirectly
during notebook rendering.

See [01-supported-rtd-build-configuration.md](01-supported-rtd-build-configuration.md)
for implementation and acceptance checks.
