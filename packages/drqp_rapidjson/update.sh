#!/usr/bin/env bash

set -e

test $(git status --porcelain | head -255 | wc -l) -eq 0 || (echo "There are uncommited changes. Commit/stash all and rerun" && exit 1)

script_dir=$(dirname "$0")

cd $script_dir
cd $(git rev-parse --show-toplevel)

prefix=packages/drqp_rapidjson/include/


if [[ -d $prefix ]]; then
    rm -rf "$prefix"
    git add "$prefix" && git commit -m "Remove old rapidjson"
fi
git subtree add --prefix="$prefix" https://github.com/Dr-QP/rapidjson.git drqp_rapidjson --squash

touch "$prefix/AMENT_IGNORE"
git add "$prefix/AMENT_IGNORE" && git commit -m "Ignore rapidjson for ament clang format"
