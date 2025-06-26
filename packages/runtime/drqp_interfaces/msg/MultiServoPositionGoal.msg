uint8 MODE_SYNC = 0 # in sync mode playtime is used from the first goal
uint8 MODE_ASYNC = 1

std_msgs/Header header
uint8 mode
drqp_interfaces/ServoPositionGoal[] goals
