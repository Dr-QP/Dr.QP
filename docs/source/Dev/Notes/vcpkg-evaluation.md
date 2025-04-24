# vcpkg evaluation

TL;DR doesn’t suite needs of the project

Notable limitations: - doesn’t provide any package co-development
support, so it not going to be possible to use packages during their
development requiring publishing of every minor change and full rebuild
by vcpkg - it is limited in support of versions. there can be just 1
version in the repo. you have to switch - only cmake as a build system,
if library doesn’t provide cmake build it has to be scripted with cmake
build steps (which is possible, but not convenient) - limited number of
platforms (mac, win, linux + uwp). Adding support for Android, iOS and
Arduino can be tricky

Pros: - serverless
