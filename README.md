# Dr.QP

[![ci](https://github.com/Dr-QP/Dr.QP/actions/workflows/ci.yml/badge.svg)](https://github.com/Dr-QP/Dr.QP/actions/workflows/ci.yml?query=branch%3Amain)
[![CodeQL Advanced](https://github.com/Dr-QP/Dr.QP/actions/workflows/codeql.yml/badge.svg)](https://github.com/Dr-QP/Dr.QP/actions/workflows/codeql.yml?query=branch%3Amain)
[![codecov](https://codecov.io/gh/Dr-QP/Dr.QP/graph/badge.svg?token=MSNH7AK8XX)](https://codecov.io/gh/Dr-QP/Dr.QP)

## Bootstrap

1. Follow `scripts/ros/Install-ros.md` to install ROS2
2. Open VSCode from the terminal with activated `ros_env`

```bash
mkdir -p drqp_ws/src
git clone git@github.com:Dr-QP/Dr.QP.git src/Dr.QP
cd drqp_ws
ros2_activate
./src/Dr.QP/scripts/ros/ros-dep.sh
```

3. Open `/src/Dr.QP/Dr.QP.code-workspace` in VSCode
4. Install recommended VSCode extensions
