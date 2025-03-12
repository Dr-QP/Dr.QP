#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/../__utils.sh"

IMAGE=ghcr.io/dr-qp/ros-deploy
docker build -t $IMAGE --build-arg GIT_SHA=$(git log -1 --format=%H) --build-arg BUILD_IMAGE=ghcr.io/dr-qp/$ROS_DISTRO-ros-desktop:pr-49 -f "$script_dir/ros-deploy.dockerfile" "$script_dir"
