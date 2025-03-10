#!/usr/bin/env bash
set -e

script_dir="$(dirname $0)"
source "$script_dir/__utils.sh"

if isCI; then
  set -x
fi


mkdir -p ~/ros2_$ROS_DISTRO/src
cd ~/ros2_$ROS_DISTRO
vcs import --input https://raw.githubusercontent.com/ros2/ros2/$ROS_DISTRO/ros2.repos src
vcs pull src

rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers ignition-cmake2 ignition-math6"

cd ~/ros2_$ROS_DISTRO/
colcon build --symlink-install --packages-skip image_tools intra_process_demo
