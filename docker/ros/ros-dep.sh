#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

# -r is needed for arm64 install as it doesn't have gazebo
rosdep install --from-paths "$sources_dir" --ignore-src -y -r
