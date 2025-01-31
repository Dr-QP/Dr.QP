#!/usr/bin/env bash
# -*- coding: utf-8 -*-

export MAMBA_NO_BANNER=1

function ros2_activate
{
  export ROS_DOMAIN_ID=3
  conda activate ros_env
  source $CONDA_PREFIX/setup.bash
}

function ros2_ws
{
    ros2_activate
    source setup.bash
}

export -f ros2_activate
export -f ros2_ws
