# Dr.QP

## Bootstrap

1. Follow `scripts/ros/Install-ros.md` to install ROS2 via conda
2. Open VSCOde from the terminal with activated `ros_env`

```
cd DR.QP
ros2_ws
rosdep install --from-paths src --ignore-src -y
```

3. Open VSCode using `code .` and install recomemnded extensions
4. Run `CMake: Scan for kits` from VSCode command pallet to add compilers from ros env

### purecpp bootstrap

```
sudo apt update && sudo apt install -y clang clangd ninja-build cmake build-essential \
    doctest

```