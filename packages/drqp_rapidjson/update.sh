#!/usr/bin/env bash

script_dir=$(dirname "$0")

cd $script_dir
cd $(git rev-parse --show-toplevel)

prefix=packages/drqp_rapidjson/include/

if [[ -d $prefix ]]; then
    rm -rf "$prefix"
    git add "$prefix" && git commit -m "Remove old rapidjson"
fi
git subtree add --prefix="$prefix" https://github.com/Dr-QP/rapidjson.git drqp_rapidjson --squash
git subtree pull --prefix="$prefix" https://github.com/Dr-QP/rapidjson.git drqp_rapidjson --squash

touch "$prefix/AMENT_IGNORE"
git add "$prefix/AMENT_IGNORE" && git commit -m "Ignore rapidjson for ament clang format"
