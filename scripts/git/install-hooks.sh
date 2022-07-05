#!/bin/bash
set -e
# set -x

script_path="${BASH_SOURCE[0]}"
script_dir=$(dirname $script_path)
git_dir=$(git rev-parse --git-dir)

echo "Discovering and linking hooks..."
find "$script_dir/hooks" -type f | while read hook; do
    echo "Linking '$hook'"
    ln -s -f "$hook" "$git_dir/hooks/"
done
echo "Done!"