#!/usr/bin/env bash
set -e
set -x

script_dir="$(dirname $0)"

# Basic prerequisites
sudo apt update \
  && sudo apt install -y -q --no-install-recommends \
  locales \
  software-properties-common \
  wget \
  curl \
  gnupg2 \
  lsb-release \
  ca-certificates

# Enable community-maintained free and open-source software (universe).
sudo add-apt-repository universe -y

# check for UTF-8
function is_utf8_locale()
{
  local loc=$(locale)
  if [[ $loc =~ LANG=.*\.UTF-8 ]] || [[ $loc =~ LC_ALL=.*\.UTF-8 ]]; then
    echo 'yes'
  else
    echo 'no'
  fi
}

if [[ $(is_utf8_locale) == 'no' ]]; then
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
fi
test $(is_utf8_locale) == 'yes' || (echo "Failed to set en_US.UTF-8 locale" && exit 1)

# Clang 19
CLANG_VERSION=19
curl -sSL https://apt.llvm.org/llvm.sh -o "$script_dir/llvm.sh"
chmod +x "$script_dir/llvm.sh"
sudo "$script_dir/llvm.sh" $CLANG_VERSION all
sudo "$script_dir/update-alternatives-clang.sh" $CLANG_VERSION 99999

# Add ros signing keys
if [[ ! (-f /etc/apt/sources.list.d/ros2-latest.list || -f /etc/apt/sources.list.d/ros2.list) ]]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# CMake
test -f /usr/share/doc/kitware-archive-keyring/copyright || wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt update
test -f /usr/share/doc/kitware-archive-keyring/copyright || sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
sudo apt install -y -q --no-install-recommends kitware-archive-keyring

sudo apt upgrade -y -q

sudo apt install -y -q --no-install-recommends \
  build-essential \
  cmake \
  ninja-build \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-colcon-coveragepy-result \
  python3-coverage \
  python3-colcon-lcov-result \
  lcov \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool

# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest

# install Fast-RTPS dependencies
sudo apt install -y -q --no-install-recommends \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install -y -q --no-install-recommends \
  libcunit1-dev


sudo apt install -y -q --no-install-recommends python3-rosdep2
sudo rm -f /etc/ros/rosdep/sources.list.d/*
sudo rosdep init
rosdep update

sudo apt install -y -q --no-install-recommends ros-humble-desktop ros-dev-tools

# Install Node.js and NPM for local GHA via nektos/act to work
curl -sL https://deb.nodesource.com/setup_20.x -o $script_dir/nodesource_setup.sh
sudo bash $script_dir/nodesource_setup.sh
sudo apt-get -y remove nodejs npm # remove old versions if any
# The NodeSource nodejs package contains both the node binary and npm, so you don’t need to install npm separately.
sudo apt-get install -y -q --no-install-recommends nodejs

"$script_dir/fix-alternatives.sh"
