#!/bin/bash
# shellcheck disable=SC1090,SC1091
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

OVERLAY_PATH="${OVERLAY_WS:-/opt/ros/overlay_ws}"
if [[ -f "$OVERLAY_PATH/install/local_setup.bash" ]]; then
  source $OVERLAY_PATH/install/local_setup.bash --
fi

if [[ -f "~/ros2_ws/install/local_setup.bash" ]]; then
  source ~/ros2_ws/install/local_setup.bash --
fi

# activate python venv
if [[ -f "$OVERLAY_PATH/.venv-prod/bin/activate" ]]; then
  source $OVERLAY_PATH/.venv-prod/bin/activate
fi

exec "$@"
