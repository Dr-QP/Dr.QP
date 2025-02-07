#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

if [[ -f "~/ros2_ws/install/setup.bash" ]]; then
  source ~/ros2_ws/install/setup.bash --
fi

exec "$@"
