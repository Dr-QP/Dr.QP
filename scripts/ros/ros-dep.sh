#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

rosdep install --from-paths "$sources_dir" --ignore-src -r -y
