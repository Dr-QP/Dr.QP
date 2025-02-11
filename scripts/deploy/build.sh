#!/usr/bin/env bash

script_dir=$(dirname $0)

IMAGE=dr-qp/ros-desktop
docker build -t dr-qp/ros-desktop --build-arg $(git log -1 --format=%H) -f "$script_dir/ros-deploy.dockerfile" "$script_dir"
