#!/usr/bin/env bash

# Shared Super-Linter defaults. Callers may override the image with the
# SUPER_LINTER_IMAGE environment variable or their own command-line option.
# shellcheck disable=SC2034 # Sourced by Super-Linter wrapper scripts.
SUPER_LINTER_DEFAULT_IMAGE="ghcr.io/super-linter/super-linter:v8.5.0"

# zizmor is installed by the base image (ghcr.io/plume-works/agent-desktop), whose
# dev_tools role pins it, so there is no local ansible variable to read the version
# from. This records the version that image is expected to provide.
#
# It is a declaration, not an install: validate-super-linter-tool-versions.sh checks
# it against Super-Linter, and additionally against the real `zizmor --version`
# whenever the binary is on PATH -- which it is in the devcontainer and in any
# container job, but not on the plain runner that workflow uses. Bump this when the
# base image bumps zizmor.
# shellcheck disable=SC2034 # Sourced by Super-Linter wrapper scripts.
BASE_IMAGE_ZIZMOR_VERSION="1.22.0"
