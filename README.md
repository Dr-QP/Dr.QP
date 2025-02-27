# Dr.QP

[![ci](https://github.com/Dr-QP/Dr.QP/actions/workflows/ci.yml/badge.svg)](https://github.com/Dr-QP/Dr.QP/actions/workflows/ci.yml?query=branch%3Amain)
[![CodeQL Advanced](https://github.com/Dr-QP/Dr.QP/actions/workflows/codeql.yml/badge.svg)](https://github.com/Dr-QP/Dr.QP/actions/workflows/codeql.yml?query=branch%3Amain)
[![codecov](https://codecov.io/gh/Dr-QP/Dr.QP/graph/badge.svg?token=MSNH7AK8XX)](https://codecov.io/gh/Dr-QP/Dr.QP)

Welcome to the Dr.QP project.

[<img width="1295" alt="image" src="https://github.com/user-attachments/assets/2f102f28-425b-4f2a-b235-58e136ed7659" />](https://a360.co/4hMiK1E)

## Wiki

All the project information and notes is available on the [Project Wiki](https://github.com/Dr-QP/Dr.QP/wiki)

## Project planning

The dev backlog and work-in-progress can be found on the [Project Tab](https://github.com/orgs/Dr-QP/projects/3)

## Dev machine setup

There are several ways to setup your dev environment, but in either case you would need source code from this repo

```bash
git clone https://github.com/Dr-QP/Dr.QP.git
cd Dr.QP
```

### Devcontainer (any desktop OS)

1. Open [Dr.QP.code-workspace](Dr.QP.code-workspace) in VSCode
2. Install recommended VSCode extensions
3. Install docker (either [Docker Desktop](https://www.docker.com/products/docker-desktop/) for macOS and Windows) or native docker installtion for linux [scripts/install_docker.sh](scripts/install_docker.sh)
4. Run `Dev Containers: Open Workspace in Container...` via command pallet `F1` and select [Dr.QP.code-workspace](Dr.QP.code-workspace) workspace
5. Choose devcontainer to open:
   - (recommended, fast) `prebuilt` to use devcontainer based on `ghcr.io/dr-qp/ros-desktop:edge` image built from `main` branch
   - `source` to build devcontainer from source in the current branch. This setup is useful if you will need to make changes to the dockerfile or installation scripts

### Raw Dev machine (linux only)

While ROS 2 is supported on many OS, the quality of support is still low and many non core packages have bugs.

Linux is the only platform that has been tested for this project.

1. Follow [Install-ros.md](scripts/ros/Install-ros.md) to install ROS2
2. Open [Dr.QP.code-workspace](Dr.QP.code-workspace) in VSCode
3. Install recommended VSCode extensions
4. Install ROS dependencies

```bash
ros2_activate # Source ROS environment
./scripts/ros/ros-dep.sh # Install all ROS dependencies
```
