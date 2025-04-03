import sensor_msgs.msg
import rclpy
import rclpy.node
import numpy as np

kFemurOffsetAngle = -13.11
kTibiaOffsetAngle = -32.9


class JointStatePublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('drqp/hexapod_joint_state_publisher')

        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, '/joint_states', qos_profile=50
        )

    def publish(self, hexapod):
        msg = sensor_msgs.msg.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for leg in hexapod.legs:
            for joint, angle in [
                ('coxa', leg.coxa_angle),
                ('femur', leg.femur_angle + kFemurOffsetAngle),
                ('tibia', leg.tibia_angle + kTibiaOffsetAngle),
            ]:
                msg.name.append(f'dr_qp/{leg.label}_{joint}')
                msg.position.append(np.radians(angle))

        self.joint_state_pub.publish(msg)
