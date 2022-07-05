#!/usr/bin/env bash
set -e
set -x

script_path="${BASH_SOURCE[0]}"
script_dir=$(dirname $script_path)
root_dir=$(dirname $script_dir)

cd "$root_dir"

env | sort
which cmake
which ninja
which $CC

rm -rf build/
mkdir build/
cd build/
cmake -G Ninja ..
ninja
