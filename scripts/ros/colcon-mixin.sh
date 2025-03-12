#!/usr/bin/env bash

script_dir=$(dirname $0)
source "$script_dir/__utils.sh"

if [[ $(colcon mixin list | grep default) != '' ]]; then
  colcon mixin remove default
  colcon mixin update
fi

colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml
colcon mixin update default
colcon mixin list
colcon mixin show
