#!/usr/bin/env bash

script_dir=$(dirname "$0")

cd $script_dir
cd $(git rev-parse --show-toplevel)

git subtree add --prefix="$script_dir/packages/drqp_rapidjson/include/" https://github.com/Dr-QP/rapidjson.git drqp_rapidjson --squash
git subtree pull --prefix="$script_dir/packages/drqp_rapidjson/include/"
