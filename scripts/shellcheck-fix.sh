#!/usr/bin/env bash

set -euo pipefail

find . \
  -path '*/vendor' -prune -o \
  -path '*/drqp_rapidjson/include' -prune -o \
  -iname '*.sh' \
  -printf '%P\0' \
  | xargs -0 shellcheck -f diff | git apply
