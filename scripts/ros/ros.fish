#!/usr/bin/env fish
# -*- coding: utf-8 -*-


set -gx MAMBA_NO_BANNER 1

function ros2_activate
    set -gx ROS_DOMAIN_ID 3
    conda activate ros_env
    bass source $CONDA_PREFIX/setup.bash
    register-python-argcomplete --shell fish ros2 | source
    register-python-argcomplete --shell fish colcon | source
    register-python-argcomplete --shell fish rosidl | source
    register-python-argcomplete --shell fish ament_index | source
end

function ros2_ws
    ros2_activate
    bass source setup.bash
end
