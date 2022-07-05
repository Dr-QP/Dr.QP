#!/bin/bash
set -e
set -x

script_path="${BASH_SOURCE[0]}"
script_dir=$(dirname $script_path)
git_dir=$(dirname $script_dir)/.git

find "$script_dir/hooks" -type f | xargs -I{} ln -s -f "{}" "$git_dir/hooks/"
