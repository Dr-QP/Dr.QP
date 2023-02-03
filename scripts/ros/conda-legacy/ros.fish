#!/usr/bin/env fish
# -*- coding: utf-8 -*-

set SCRIPT_DIR (dirname (status --current-filename))

set -gx MAMBA_NO_BANNER 1

function ros2_activate
    set -gx ROS_DOMAIN_ID 3
    set -gx PIP_CONFIG_FILE $SCRIPT_DIR/pip.conf
    conda activate ros_env
    bass source $CONDA_PREFIX/setup.bash
    register-python-argcomplete --shell fish ros2 | source
    register-python-argcomplete --shell fish colcon | source
    register-python-argcomplete --shell fish rosidl | source
    register-python-argcomplete --shell fish ament_index | source
end

function ros2_ws
    ros2_activate
    bass source install/local_setup.bash
end

function rosdep_src
    rosdep install --from-paths src --ignore-src -r -y
end
