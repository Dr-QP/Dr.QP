#!/usr/bin/env bash

gitdir="$(git rev-parse --git-common-dir)"
case $gitdir in
    /*) ;;
    *) gitdir=$PWD/$gitdir
esac

script_dir=$(dirname "${BASH_SOURCE[0]}")
echo "GIT_REPO=$(realpath "$gitdir")" > "$script_dir/.env"
