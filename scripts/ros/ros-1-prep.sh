#!/usr/bin/env bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt upgrade

# Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
# sudo apt install ros-noetic-desktop-full

# Desktop Install: Everything in ROS-Base plus tools like rqt and rviz
# sudo apt install ros-noetic-desktop

# ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools.
# sudo apt install ros-noetic-ros-base

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep

echo sudo apt install ros-noetic-ros-base
echo sudo apt install ros-noetic-desktop


