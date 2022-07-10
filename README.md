# Dr.QP

## Bootstrap

1. Follow `scripts/ros/Install-ros.md` to install ROS2 via conda
2. Open VSCOde from the terminal with activated `ros_env`

```
cd DR.QP
conda activate ros_env
code .
```

3. Open VSCode command pallet and run `CMake: Scan for kits` to add compilers from ros env

<!-- 4. Add `"environmentSetupScript": "/Users/antonmatosov/opt/miniconda3/etc/conda.sh\" ; conda activate ros_env\""` to ros compilers in `CMake: Edit User-Local CMake Kits` -->

## Notes:

# Robot Web Tools

As of now is not compatible with ROS2 Humble
https://github.com/pantor/ros-control-center/issues/33
`rosbridge_websocket]: [Client 43df0659-04ef-4217-a40c-0fd4278fca0c] [id: subscribe:/rosout:5] subscribe: Unable to import msg class Log from package rosgraph_msgs. Caused by module 'rosgraph_msgs.msg' has no attribute 'Log'`
and cascades into a stream of errors. [Full log](https://gist.github.com/anton-matosov/a8ad045569f947e07a6df24642d6b4ef)

