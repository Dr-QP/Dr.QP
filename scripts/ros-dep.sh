#!/usr/bin/env bash

script_dir=$(dirname "$0")
source "$script_dir/__utils.sh"

sudo apt-get update

source "$script_dir/setup.bash"

# pytest-retry has no rosdep key, but launch tests use @pytest.mark.flaky in
# the ROS/colcon test environment (which does not run uv sync). Install it
# explicitly so pytest loads the retry plugin in CI and local ROS test runs.
/usr/bin/python3 -m pip install --break-system-packages 'pytest-retry>=1.7.0'

# -r is needed for arm64 install as it doesn't have gazebo
rosdep install --from-paths "$sources_dir" --ignore-src -y -r

"$root_dir/docker/ros/deploy/install-overlay-python-requirements.py" \
  "$root_dir/build" \
  "$root_dir/install"
