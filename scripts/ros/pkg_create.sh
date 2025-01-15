#!/usr/bin/env bash

script_dir=$(dirname $0)
root_dir=$(dirname $(dirname $script_dir))
sources_dir="$root_dir/src"

ros2 pkg create \
  --license MIT \
  --build-type ament_cmake \
  --maintainer-email anton.matosov@gmail.com \
  --maintainer-name "Anton Matosov" \
  --destination-directory "$sources_dir" \
  "$@"
