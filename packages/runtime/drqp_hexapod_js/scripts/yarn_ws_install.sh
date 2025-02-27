#!/usr/bin/env bash
# set -x
set -e

local_setup=$1

source $local_setup

yarn install
