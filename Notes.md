

## Notes:

# Robot Web Tools
http://robotwebtools.org/

## Foxglove Studio - WORKS!!
Advanced web viewer. Has native app as well
https://foxglove.dev/studio

https://github.com/foxglove/studio

## Rosboard - works
A simplistic web
https://github.com/dheera/rosboard

## ros-control-center does not work
As of now is not compatible with any ROS2
https://github.com/pantor/ros-control-center/issues/33
`rosbridge_websocket]: [Client 43df0659-04ef-4217-a40c-0fd4278fca0c] [id: subscribe:/rosout:5] subscribe: Unable to import msg class Log from package rosgraph_msgs. Caused by module 'rosgraph_msgs.msg' has no attribute 'Log'`
and cascades into a stream of errors. [Full log](https://gist.github.com/anton-matosov/a8ad045569f947e07a6df24642d6b4ef)


