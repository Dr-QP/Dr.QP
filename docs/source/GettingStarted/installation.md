# Dev machine setup

There are several ways to setup your dev environment, but in either case you would need source code from this repo

```bash
git clone https://github.com/Dr-QP/Dr.QP.git
cd Dr.QP
```

## Devcontainer (any desktop OS)

1. Open `Dr.QP.code-workspace` in VSCode
2. Install recommended VSCode extensions
3. Run the task `Dr.QP workspace extensions` to clone local workspace extensions, including `ros2-xunit-viewer`
4. Install docker (either [Docker Desktop](https://www.docker.com/products/docker-desktop/) for macOS and Windows) or native docker installation for linux `scripts/install_docker.sh`
5. Run `Dev Containers: Reopen in Container` via command pallet `F1` and select `Dr.QP.code-workspace` workspace
6. Choose devcontainer to open:

After running tests, use the command palette action `ROS2: Open XUnit Test Results` to browse `colcon` results inside VS Code.

## Raw Dev machine (linux only)

While ROS 2 is supported on many OS, the quality of support is still low and many non core packages have bugs.

Linux is the only platform that has been tested for this project.

1. Use Ubuntu 24.04 Noble as the base OS
2. Open `Dr.QP.code-workspace` in VSCode
3. Install recommended VSCode extensions
4. Run the task `Dr.QP workspace extensions` to clone local workspace extensions, including `ros2-xunit-viewer`
5. Create venv using the task `Dr.QP venv`
6. Setup environment using the task `Dr.QP setup ROS`
7. Install ROS dependencies using the task `Dr.QP rosdep`

After running tests, use the command palette action `ROS2: Open XUnit Test Results` to browse `colcon` results inside VS Code.
