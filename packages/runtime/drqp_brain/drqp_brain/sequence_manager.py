# Copyright (c) 2017-2025 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from drqp_brain.timed_queue import TimedQueue
import std_msgs.msg


class SequenceManager:
    """
    Manages initialization and finalization sequences for the hexapod robot.
    
    This class encapsulates the logic for robot startup and shutdown sequences,
    making them easier to test and maintain.
    """

    def __init__(self, node, hexapod, trajectory_publisher, event_publisher, logger):
        """
        Initialize the sequence manager.
        
        Parameters
        ----------
        node : rclpy.node.Node
            ROS node for timer creation
        hexapod : HexapodModel
            The hexapod model for kinematics
        trajectory_publisher : callable
            Function to publish joint trajectories
        event_publisher : rclpy.publisher.Publisher
            Publisher for robot events
        logger : rclpy.logger.Logger
            Logger for sequence messages
        """
        self.hexapod = hexapod
        self.publish_joint_position_trajectory = trajectory_publisher
        self.robot_event_pub = event_publisher
        self.logger = logger
        self.sequence_queue = TimedQueue(node)

    def start_initialization_sequence(self):
        """Start the robot initialization sequence."""
        self.sequence_queue.clear()
        self.sequence_queue.add(1.0, self._initialization_step1)
        self.sequence_queue.add(0.6, self._initialization_step2)
        self.sequence_queue.add(0.6, self._initialization_step3)
        self.sequence_queue.add(0.6, self._initialization_step4)
        self.sequence_queue.add(0.5, self._initialization_step5)
        self.sequence_queue.add(0.0, self._initialization_done)

    def start_finalization_sequence(self):
        """Start the robot finalization sequence."""
        self.sequence_queue.clear()
        self.sequence_queue.add(1.1, self._finalization_step1)
        self.sequence_queue.add(0.6, self._finalization_step2)
        self.sequence_queue.add(0.0, self._finalization_done)

    def clear_sequence(self):
        """Clear any running sequence."""
        self.sequence_queue.clear()

    # Initialization sequence steps
    def _initialization_step1(self):
        """Turn torque on for femur and move all femur to -105."""
        self.logger.info('Initialization sequence started')
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=900, joint_mask=['femur'])

    def _initialization_step2(self):
        """Turn torque on for tibia and move all tibia to 0."""
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=500, joint_mask=['femur', 'tibia'])

    def _initialization_step3(self):
        """Turn torque on for coxa and move all coxa to 0."""
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=500)

    def _initialization_step4(self):
        """Move all tibia to 95."""
        self.hexapod.forward_kinematics(0, -105, 95)
        self.publish_joint_position_trajectory(playtime_ms=500)

    def _initialization_step5(self):
        """Use walk controller to move to default position slowly increasing body height."""
        self.hexapod.forward_kinematics(0, -35, 130)
        self.publish_joint_position_trajectory(playtime_ms=400)

    def _initialization_done(self):
        """Signal that initialization is complete."""
        self.robot_event_pub.publish(std_msgs.msg.String(data='initializing_done'))

    # Finalization sequence steps
    def _finalization_step1(self):
        """Move to safe retracted position."""
        self.logger.info('Finalization sequence started')
        self.hexapod.forward_kinematics(0, -105, 0)
        self.publish_joint_position_trajectory(playtime_ms=1000)

    def _finalization_step2(self):
        """Move to fully retracted position."""
        self.hexapod.forward_kinematics(0, -105, -60)
        self.publish_joint_position_trajectory(playtime_ms=500)

    def _finalization_done(self):
        """Signal that finalization is complete."""
        self.robot_event_pub.publish(std_msgs.msg.String(data='finalizing_done'))

    # Utility methods for testing
    def get_initialization_steps(self):
        """Get the list of initialization steps for testing."""
        return [
            (1.0, self._initialization_step1),
            (0.6, self._initialization_step2),
            (0.6, self._initialization_step3),
            (0.6, self._initialization_step4),
            (0.5, self._initialization_step5),
            (0.0, self._initialization_done),
        ]

    def get_finalization_steps(self):
        """Get the list of finalization steps for testing."""
        return [
            (1.1, self._finalization_step1),
            (0.6, self._finalization_step2),
            (0.0, self._finalization_done),
        ]
