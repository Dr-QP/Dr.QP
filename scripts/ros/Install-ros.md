# Installing ROS

## Development machine

For the development machine use Ubuntu 22.04 and install ros using ros-2-prep.sh. At the end script will print installation instruction for base or desktop distro.
Run the base or desktop installation manually

### Bash

Update your `~/.bashrc` with the following snippet that adds couple alises for easy activation of ros1 and 2 globally or in workspace

```bash
#################################################################
if [ -z "$ROS_IP" ]; then
  export ROS_IP=127.0.0.1
fi

alias ros1_activate="source /opt/ros/noetic/setup.bash"
alias ros1_ws="source ./devel/setup.bash"

alias ros2_activate="source /opt/ros/humble/setup.bash"
alias ros2_ws="source ./install/setup.bash"

```

### Fish

For `fish` shell users run setup script

```bash
./scripts/ros/fish/setup.fish
```
