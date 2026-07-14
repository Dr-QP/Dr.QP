#!/usr/bin/env bash

# Shared Super-Linter defaults. Callers may override the image with the
# SUPER_LINTER_IMAGE environment variable or their own command-line option.
# shellcheck disable=SC2034 # Sourced by Super-Linter wrapper scripts.
SUPER_LINTER_DEFAULT_IMAGE="ghcr.io/super-linter/super-linter:v8.5.0"
