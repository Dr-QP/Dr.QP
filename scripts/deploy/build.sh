#!/usr/bin/env bash

script_dir=$(dirname $0)

IMAGE=ghcr.io/dr-qp/ros-deploy
docker build -t $IMAGE --build-arg GIT_SHA=$(git log -1 --format=%H) -f "$script_dir/ros-deploy.dockerfile" "$script_dir"
