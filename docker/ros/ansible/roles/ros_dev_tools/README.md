# ros_dev_tools

Installs the ROS-specific apt packages this workspace builds and tests with:
the colcon extensions, the `ament_flake8` plugin set, the pytest plugins,
`ros-dev-tools`, `rosdep`, `vcstool`, and the C++ libraries ROS 2 packages
link against.

This role is deliberately only the **delta** over the base image
(`ghcr.io/plume-works/agent-desktop`), whose `dev_tools` role supplies the
generic toolchain and the pinned single-binary tools. Before adding a package
here, check it is not already in the base image's `dev_tools` list — a
duplicate installs fine but hides drift between the two.

## Requirements

Run after `ros_repo` and `osrf_repo`: `ros-dev-tools` and the `python3-colcon-*`
packages come from the ROS 2 apt repository, and `python3-gz-transport13` from
the OSRF repository.

## Role Variables

None.
