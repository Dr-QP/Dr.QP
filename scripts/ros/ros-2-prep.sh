#!/usr/bin/env bash
set -e
set -x

script_dir="$(dirname $0)"

locale  # check for UTF-8

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y wget curl gnupg2 lsb-release ca-certificates

# Clang 19
CLANG_VERSION=19
curl -sSL https://apt.llvm.org/llvm.sh -o "$script_dir/llvm.sh"
chmod +x "$script_dir/llvm.sh"
sudo "$script_dir/llvm.sh" $CLANG_VERSION all
sudo "$script_dir/update-alternatives-clang.sh" $CLANG_VERSION 1


if [[ ! (-f /etc/apt/sources.list.d/ros2-latest.list || -f /etc/apt/sources.list.d/ros2.list) ]]; then
  # sudo rm /etc/apt/sources.list.d/ros2-latest.list # remove old ros2-latest.list from the docker image
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

# CMake
test -f /usr/share/doc/kitware-archive-keyring/copyright || wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

sudo apt update
test -f /usr/share/doc/kitware-archive-keyring/copyright || sudo rm /usr/share/keyrings/kitware-archive-keyring.gpg
sudo apt install kitware-archive-keyring

sudo apt upgrade -y

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-colcon-coveragepy-result \
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
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev


sudo apt install -y python3-rosdep2
sudo rm -f /etc/ros/rosdep/sources.list.d/*
sudo rosdep init
rosdep update

sudo apt install -y ros-humble-desktop
# sudo apt install ros-humble-ros-base
sudo apt install -y ros-dev-tools

# Install Node.js and NPM for local GHA via nektos/act to work
curl -sL https://deb.nodesource.com/setup_20.x -o $script_dir/nodesource_setup.sh
sudo bash $script_dir/nodesource_setup.sh
sudo apt-get -y remove nodejs npm # remove old versions if any
# The NodeSource nodejs package contains both the node binary and npm, so you don’t need to install npm separately.
sudo apt-get install -y nodejs


