#!/usr/bin/env bash
set -e

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

$script_dir/ros-2-prep.sh

$script_dir/fish/setup.fish
$script_dir/colcon-mixin.sh
$script_dir/rosdep-update.sh
