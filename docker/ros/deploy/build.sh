#!/usr/bin/env bash

ROS_DISTRO=jazzy
IMAGE=ghcr.io/dr-qp/ros-deploy

script_dir=$(dirname $0)

docker build -t $IMAGE --build-arg GIT_SHA=$(git log -1 --format=%H) --build-arg BUILD_IMAGE=ghcr.io/dr-qp/$ROS_DISTRO-ros-desktop:edge -f "$script_dir/ros-deploy.dockerfile" "$script_dir"
