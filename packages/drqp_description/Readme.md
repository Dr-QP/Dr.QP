Sample commands 
```
ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data:[0.2, 0.2, 0.2], layout: {dim:[], data_offset: 1}}"

ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data:[0.5, 0.5, 0.5], layout: {dim:[], data_offset: 1}}"

ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data:[1, 1, 1], layout: {dim:[], data_offset: 1}}"

ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data:[0, 0, 0], layout: {dim:[], data_offset: 1}}"
```