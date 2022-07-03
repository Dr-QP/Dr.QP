# Ros Humble

Dr.QP project is using ROS2 Humble version
Requires Ubuntu 22.04, python 3.9

# Installation through Conda

## Cleanup old environment variables

If you have ever attempted to install ROS2 manually, you probable have some environment variables in
 - .bashrc
 - .bash_profile
 - .config/fish/config.fish
 - or other RC/config file for your shell

In order to make ROS2 setup via conda properly work you need to remove these variables from the global scope

## Remaining issues: https://github.com/RoboStack/ros-humble/issues/3

The macOS installation is using RoboStack
https://robostack.github.io/GettingStarted.html

```
# if you don't have mamba yet, install it first (not needed when using mambaforge):
conda install mamba -c conda-forge

# now create a new environment
mamba create -n ros_env python=3.9
conda activate ros_env

# this adds the conda-forge channel to the new created environment configuration 
conda config --env --add channels conda-forge
# and the robostack channels
conda config --env --add channels robostack-experimental
conda config --env --add channels robostack
conda config --env --add channels robostack-humble

# Install the version of ROS you are interested in:
mamba install ros-humble-desktop-full

# optionally, install some compiler packages if you want to e.g. build packages in a colcon_ws:
mamba install compilers cmake pkg-config make ninja colcon-common-extensions

# on Windows, install Visual Studio 2017 or 2019 with C++ support 
# see https://docs.microsoft.com/en-us/cpp/build/vscpp-step-0-installation?view=msvc-160

# on Windows, install the Visual Studio command prompt:
mamba install vs2019_win-64

# note that in this case, you should also install the necessary dependencies with conda/mamba, if possible

# reload environment to activate required scripts before running anything
# on Windows, please restart the Anaconda Prompt / Command Prompt!
conda deactivate
conda activate ros_env


## This was required to avoid sudo for rosdep
sudo mkdir /etc/ros
sudo chown -R (whoami) /etc/ros

# if you want to use rosdep, also do:
mamba install rosdep
rosdep init  # note: do not use sudo!
rosdep update
```

## Activating

In order to activate ROS2 shell 2 things need to be done:
 - `conda activate ros_env` - activate conda environment 
 - `source $CONDA_PREFIX/setup.bash` - source ROS2 specific environment variables 

### Fish convenient function

Below are a convenient functions for `fish` shell to source ROS2 environment
NOTE: it makes use of [`bass`](https://github.com/edc/bass) plugin that allows to source bash scripts in fish

```
function ros2_activate
    conda activate ros_env
    bass source $CONDA_PREFIX/setup.bash
end

function ros2_ws
    ros2_activate
    bass source setup.bash
end
```