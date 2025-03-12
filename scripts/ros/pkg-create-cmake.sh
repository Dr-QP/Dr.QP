#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

ros2 pkg create \
  --license MIT \
  --build-type ament_cmake \
  --maintainer-email anton.matosov@gmail.com \
  --maintainer-name "Anton Matosov" \
  --destination-directory "$sources_dir/runtime" \
  "$@"
