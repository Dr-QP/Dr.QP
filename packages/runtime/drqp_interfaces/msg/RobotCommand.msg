# RobotCommand.msg
# High-level robot control commands

# Message header with timestamp
std_msgs/Header header

# Command identifier (e.g., "initialize", "finalize", "reboot_servos", "kill_switch")
string command
