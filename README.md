# Dr.QP

## Bootstrap

1. Follow `scripts/ros/Install-ros.md` to install ROS2 via conda
2. Configure `"cmake.cmakePath"` in `.vscode/settings` of your workspace:

```
conda activate ros_env
which cmake
```
and put this path into
```
{
  "cmake.cmakePath": "/Users/antonmatosov/opt/miniconda3/envs/ros_env/bin/cmake"
}
```