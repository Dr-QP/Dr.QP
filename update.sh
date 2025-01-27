#!/usr/bin/env bash

script_dir=$(dirname "$0")

cd $script_dir
cd $(git rev-parse --show-toplevel)

prefix=packages/drqp_rapidjson/include/

git subtree add --prefix="$prefix" https://github.com/Dr-QP/rapidjson.git drqp_rapidjson --squash
git subtree pull --prefix="$prefix"
