#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$script_dir/__utils.sh"
# shellcheck source=/dev/null
source "$script_dir/setup.bash"

# rosdep resolves package.xml declarations and asks apt only for dependencies
# that are not already installed. -r is needed on arm64, which lacks Gazebo.
# shellcheck disable=SC2154 # __utils.sh exports sources_dir.
rosdep install --from-paths "$sources_dir" --ignore-src -y -r

# A build may generate requires.txt metadata for Python package runtime
# dependencies. The installer is intentionally a no-op before such metadata
# exists and lets pip keep already-satisfied requirements in place.
# shellcheck disable=SC2154 # __utils.sh exports root_dir.
"$root_dir/docker/ros/deploy/install-overlay-python-requirements.py" \
  "$root_dir/build" \
  "$root_dir/install"
