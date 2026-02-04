# MovementCommand.msg
# Semantic robot movement command abstraction

# Direction of robot movement (normalized -1.0 to 1.0 for each axis)
# x: forward/back, y: left/right, z: up/down
geometry_msgs/Vector3 stride_direction

# Rotation speed (normalized -1.0 to 1.0, positive = counterclockwise)
float32 rotation_speed

# Body position offset relative to neutral stance (normalized -1.0 to 1.0)
geometry_msgs/Vector3 body_translation

# Body orientation (roll, pitch, yaw) in normalized units -1.0 to 1.0
geometry_msgs/Vector3 body_rotation

# Gait type identifier (e.g., "tripod", "ripple", "wave")
string gait_type
