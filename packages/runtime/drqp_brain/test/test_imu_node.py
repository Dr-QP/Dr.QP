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

import unittest
from unittest import mock

from drqp_brain.imu_node import (
    _as_quaternion,
    _as_vector3,
    ImuNode,
    ImuSample,
    main,
    SensorInitializationError,
)
import rclpy
from sensor_msgs.msg import Imu, MagneticField, Temperature


class TestImuReadingConversion(unittest.TestCase):
    """Test normalization helpers for raw IMU sensor readings."""

    def test_as_vector3_returns_none_when_any_axis_is_missing(self):
        """Treat partially missing 3-axis readings as unavailable."""
        self.assertIsNone(_as_vector3((1.0, None, 3.0)))

    def test_as_quaternion_returns_none_when_any_axis_is_missing(self):
        """Treat partially missing quaternion readings as unavailable."""
        self.assertIsNone(_as_quaternion((1.0, 0.0, None, 0.0)))


class FakeImuSensor:
    """A fake IMU sensor used to drive deterministic tests."""

    def __init__(
        self,
        sample: ImuSample | None = None,
        error: Exception | None = None,
    ):
        self.sample = sample
        self.error = error

    def read_sample(self) -> ImuSample:
        """Return the configured sample or raise the configured error."""
        if self.error is not None:
            raise self.error
        if self.sample is None:
            raise RuntimeError('A sample must be configured for the fake IMU sensor')
        return self.sample


class TestImuNode(unittest.TestCase):
    """Test the IMU publisher node with a fake sensor backend."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        sample = ImuSample(
            orientation_wxyz=(1.0, 0.1, 0.2, 0.3),
            angular_velocity=(0.4, 0.5, 0.6),
            linear_acceleration=(1.1, 1.2, 1.3),
            magnetic_field_microtesla=(11.0, 12.0, 13.0),
            temperature_celsius=24.5,
        )
        self.fake_sensor = FakeImuSensor(sample=sample)
        self.sensor_patch = mock.patch(
            'drqp_brain.imu_node.Bno055Sensor',
            return_value=self.fake_sensor,
        )
        self.addCleanup(self.sensor_patch.stop)
        self.sensor_patch.start()
        self.node = ImuNode()
        self.node.timer.cancel()
        self.test_node = rclpy.create_node('test_imu_consumer')

        self.imu_messages = []
        self.magnetic_field_messages = []
        self.temperature_messages = []

        self.imu_subscription = self.test_node.create_subscription(
            Imu, '/imu/data', lambda msg: self.imu_messages.append(msg), 10
        )
        self.magnetic_field_subscription = self.test_node.create_subscription(
            MagneticField,
            '/imu/mag',
            lambda msg: self.magnetic_field_messages.append(msg),
            10,
        )
        self.temperature_subscription = self.test_node.create_subscription(
            Temperature,
            '/imu/temperature',
            lambda msg: self.temperature_messages.append(msg),
            10,
        )

    def tearDown(self):
        self.node.destroy_node()
        self.test_node.destroy_node()

    def _spin_until(self, predicate, iterations: int = 10) -> bool:
        """Spin both nodes until the predicate is satisfied or the budget is exhausted."""
        for _ in range(iterations):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _wait_for_subscribers(self):
        """Wait until the IMU publishers have active subscriptions."""
        connected = self._spin_until(
            lambda: (
                self.node.imu_pub.get_subscription_count() > 0
                and self.node.magnetic_field_pub.get_subscription_count() > 0
                and self.node.temperature_pub.get_subscription_count() > 0
            ),
            iterations=20,
        )
        self.assertTrue(connected, 'Expected IMU subscriptions to connect')
        self.imu_messages.clear()
        self.magnetic_field_messages.clear()
        self.temperature_messages.clear()

    def test_publish_measurements_publishes_imu_and_9dof_topics(self):
        """Publish IMU, magnetic field, and temperature messages from one sample."""
        self._wait_for_subscribers()

        self.node.publish_measurements()
        delivered = self._spin_until(
            lambda: (
                len(self.imu_messages) > 0
                and len(self.magnetic_field_messages) > 0
                and len(self.temperature_messages) > 0
            ),
            iterations=20,
        )

        self.assertTrue(delivered, 'Expected IMU messages to be delivered')

        imu_msg = self.imu_messages[-1]
        self.assertEqual(imu_msg.header.frame_id, 'drqp/imu_link')
        self.assertEqual(imu_msg.orientation.w, 1.0)
        self.assertEqual(imu_msg.orientation.x, 0.1)
        self.assertEqual(imu_msg.orientation.y, 0.2)
        self.assertEqual(imu_msg.orientation.z, 0.3)
        self.assertEqual(imu_msg.angular_velocity.x, 0.4)
        self.assertEqual(imu_msg.angular_velocity.y, 0.5)
        self.assertEqual(imu_msg.angular_velocity.z, 0.6)
        self.assertEqual(imu_msg.linear_acceleration.x, 1.1)
        self.assertEqual(imu_msg.linear_acceleration.y, 1.2)
        self.assertEqual(imu_msg.linear_acceleration.z, 1.3)

        magnetic_field_msg = self.magnetic_field_messages[-1]
        self.assertEqual(magnetic_field_msg.header.frame_id, 'drqp/imu_link')
        self.assertAlmostEqual(magnetic_field_msg.magnetic_field.x, 11.0e-6)
        self.assertAlmostEqual(magnetic_field_msg.magnetic_field.y, 12.0e-6)
        self.assertAlmostEqual(magnetic_field_msg.magnetic_field.z, 13.0e-6)

        temperature_msg = self.temperature_messages[-1]
        self.assertEqual(temperature_msg.header.frame_id, 'drqp/imu_link')
        self.assertEqual(temperature_msg.temperature, 24.5)

    def test_publish_measurements_skips_publish_on_sensor_failure(self):
        """Avoid publishing stale ROS messages when a sensor read fails."""
        self.fake_sensor.error = RuntimeError('i2c read failed')
        self._wait_for_subscribers()

        self.node.publish_measurements()
        self._spin_until(lambda: False, iterations=5)

        self.assertEqual(self.imu_messages, [])
        self.assertEqual(self.magnetic_field_messages, [])
        self.assertEqual(self.temperature_messages, [])

    def test_publish_measurements_marks_orientation_unavailable_when_none(self):
        """Publish gyro/accel even when orientation is absent; mark it unavailable."""
        self.fake_sensor.sample = ImuSample(
            orientation_wxyz=None,
            angular_velocity=(0.7, 0.8, 0.9),
            linear_acceleration=(2.1, 2.2, 2.3),
            magnetic_field_microtesla=(5.0, 6.0, 7.0),
            temperature_celsius=25.0,
        )
        self._wait_for_subscribers()

        self.node.publish_measurements()
        delivered = self._spin_until(lambda: len(self.imu_messages) > 0, iterations=20)

        self.assertTrue(delivered, 'Expected IMU message to be delivered without orientation')

        imu_msg = self.imu_messages[-1]
        self.assertEqual(
            imu_msg.orientation_covariance[0],
            -1.0,
            'orientation_covariance[0] must be -1 when orientation is unavailable',
        )
        self.assertEqual(imu_msg.angular_velocity.x, 0.7)
        self.assertEqual(imu_msg.angular_velocity.y, 0.8)
        self.assertEqual(imu_msg.angular_velocity.z, 0.9)
        self.assertEqual(imu_msg.angular_velocity_covariance[0], -1.0)
        self.assertEqual(imu_msg.linear_acceleration.x, 2.1)
        self.assertEqual(imu_msg.linear_acceleration.y, 2.2)
        self.assertEqual(imu_msg.linear_acceleration.z, 2.3)
        self.assertEqual(imu_msg.linear_acceleration_covariance[0], -1.0)


class TestImuNodeInitialization(unittest.TestCase):
    """Test IMU node startup failures caused by backend construction."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_constructor_wraps_sensor_construction_failures(self):
        """Report backend construction failures with an actionable startup error."""
        with (
            mock.patch(
                'drqp_brain.imu_node.Bno055Sensor',
                side_effect=AttributeError('platform detection failed'),
            ),
            self.assertRaisesRegex(
                SensorInitializationError,
                'Failed to initialize the BNO055 IMU backend at I2C address 0x28',
            ) as context,
        ):
            ImuNode()

        self.assertIsInstance(context.exception.__cause__, AttributeError)


class TestImuMain(unittest.TestCase):
    """Test CLI behavior for IMU node startup failures."""

    def test_main_exits_cleanly_when_sensor_backend_cannot_start(self):
        """Exit with code 1 and log the startup error without a traceback."""
        logger = mock.Mock()
        imu_node_constructor = mock.Mock(
            side_effect=SensorInitializationError('backend unavailable')
        )

        with (
            mock.patch('drqp_brain.imu_node.rclpy.init'),
            mock.patch('drqp_brain.imu_node.ImuNode', imu_node_constructor),
            mock.patch('drqp_brain.imu_node.rclpy.logging.get_logger', return_value=logger),
            mock.patch('drqp_brain.imu_node.rclpy.ok', return_value=True),
            mock.patch('drqp_brain.imu_node.rclpy.shutdown') as shutdown,
        ):
            with self.assertRaises(SystemExit) as context:
                main()

        self.assertEqual(context.exception.code, 1)
        imu_node_constructor.assert_called_once_with()
        logger.error.assert_called_once_with('backend unavailable')
        shutdown.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
