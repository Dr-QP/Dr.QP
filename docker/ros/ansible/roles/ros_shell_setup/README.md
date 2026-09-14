# ros_shell_setup

Adds this workspace's ROS shell integration to bash and fish:

- `ros2_activate` and `ros2_ws` aliases in `.bashrc`;
- `~/.config/fish/conf.d/ros.fish`, providing `ros2_activate`, `ros2_ws`,
  `xpra_display`, and `ros2`/`colcon`/`rosidl`/`ament_index` tab completion.

This role is the ROS **delta** over the base image
(`ghcr.io/plume-works/agent-desktop`), whose `bash_setup` and `fish_setup`
roles install fish, fisher and bass and drop their own `dev.fish`. It replaces
the local `bash_setup` and `fish_setup` roles, which mixed that generic setup
with these ROS pieces.

## Requirements

The base image's `fish_setup` must have run, because `ros.fish` sources ROS's
bash setup scripts through `bass`.

## Role Variables

| Variable     | Source        | Purpose                                     |
| ------------ | ------------- | ------------------------------------------- |
| `ros_distro` | `group_vars`  | ROS distribution sourced by `ros2_activate` |
| `user_home`  | `extra_facts` | Home directory the config is written into   |
