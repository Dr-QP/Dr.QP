#!/usr/bin/env bash
set -e

script_dir=$(dirname $0)
source "$script_dir/ros/__utils.sh"

$script_dir/ros-2-prep.sh
$script_dir/colcon-build.sh
$script_dir/rosdep-update.sh
$script_dir/ros-dep.sh
