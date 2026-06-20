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

from dataclasses import dataclass, field
from unittest import mock

from drqp_brain.imu_node import (
    _as_quaternion,
    _as_vector3,
    ImuNode,
    ImuSample,
    main,
    SensorInitializationError,
)
import pytest
import rclpy
from sensor_msgs.msg import Imu, MagneticField, Temperature


def test_as_vector3_returns_none_when_any_axis_is_missing():
    """Treat partially missing 3-axis readings as unavailable."""
    assert _as_vector3((1.0, None, 3.0)) is None


def test_as_quaternion_returns_none_when_any_axis_is_missing():
    """Treat partially missing quaternion readings as unavailable."""
    assert _as_quaternion((1.0, 0.0, None, 0.0)) is None


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


@dataclass
class _ImuHarness:
    """Bundle the IMU node under test, its fake sensor, and captured messages."""

    node: ImuNode
    test_node: 'rclpy.node.Node'
    fake_sensor: FakeImuSensor
    imu_messages: list = field(default_factory=list)
    magnetic_field_messages: list = field(default_factory=list)
    temperature_messages: list = field(default_factory=list)

    def spin_until(self, predicate, iterations: int = 10) -> bool:
        """Spin both nodes until the predicate is satisfied or the budget is exhausted."""
        for _ in range(iterations):
            rclpy.spin_once(self.node, timeout_sec=0.02)
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def wait_for_subscribers(self) -> None:
        """Wait until the IMU publishers have active subscriptions."""
        connected = self.spin_until(
            lambda: (
                self.node.imu_pub.get_subscription_count() > 0
                and self.node.magnetic_field_pub.get_subscription_count() > 0
                and self.node.temperature_pub.get_subscription_count() > 0
            ),
            iterations=20,
        )
        assert connected, 'Expected IMU subscriptions to connect'
        self.imu_messages.clear()
        self.magnetic_field_messages.clear()
        self.temperature_messages.clear()


@pytest.fixture
def rclpy_context():
    """Provide a ROS context for tests that construct ROS nodes directly."""
    rclpy.init()
    try:
        yield
    finally:
        rclpy.try_shutdown()


@pytest.fixture
def imu_harness(rclpy_context):  # noqa: ARG001 (needs rclpy)
    """Provide an IMU node backed by a fake sensor wired to a consumer node."""
    sample = ImuSample(
        orientation_wxyz=(1.0, 0.1, 0.2, 0.3),
        angular_velocity=(0.4, 0.5, 0.6),
        linear_acceleration=(1.1, 1.2, 1.3),
        magnetic_field_microtesla=(11.0, 12.0, 13.0),
        temperature_celsius=24.5,
    )
    fake_sensor = FakeImuSensor(sample=sample)
    with mock.patch('drqp_brain.imu_node.Bno055Sensor', return_value=fake_sensor):
        node = ImuNode()
        node.timer.cancel()
        test_node = rclpy.create_node('test_imu_consumer')
        harness = _ImuHarness(node=node, test_node=test_node, fake_sensor=fake_sensor)

        test_node.create_subscription(
            Imu, '/imu/data', lambda msg: harness.imu_messages.append(msg), 10
        )
        test_node.create_subscription(
            MagneticField,
            '/imu/mag',
            lambda msg: harness.magnetic_field_messages.append(msg),
            10,
        )
        test_node.create_subscription(
            Temperature,
            '/imu/temperature',
            lambda msg: harness.temperature_messages.append(msg),
            10,
        )

        try:
            yield harness
        finally:
            test_node.destroy_node()
            node.destroy_node()


def test_publish_measurements_publishes_imu_and_9dof_topics(imu_harness):
    """Publish IMU, magnetic field, and temperature messages from one sample."""
    imu_harness.wait_for_subscribers()

    imu_harness.node.publish_measurements()
    delivered = imu_harness.spin_until(
        lambda: (
            len(imu_harness.imu_messages) > 0
            and len(imu_harness.magnetic_field_messages) > 0
            and len(imu_harness.temperature_messages) > 0
        ),
        iterations=20,
    )

    assert delivered, 'Expected IMU messages to be delivered'

    imu_msg = imu_harness.imu_messages[-1]
    assert imu_msg.header.frame_id == 'drqp/imu_link'
    assert imu_msg.orientation.w == 1.0
    assert imu_msg.orientation.x == 0.1
    assert imu_msg.orientation.y == 0.2
    assert imu_msg.orientation.z == 0.3
    assert imu_msg.angular_velocity.x == 0.4
    assert imu_msg.angular_velocity.y == 0.5
    assert imu_msg.angular_velocity.z == 0.6
    assert imu_msg.linear_acceleration.x == 1.1
    assert imu_msg.linear_acceleration.y == 1.2
    assert imu_msg.linear_acceleration.z == 1.3

    magnetic_field_msg = imu_harness.magnetic_field_messages[-1]
    assert magnetic_field_msg.header.frame_id == 'drqp/imu_link'
    assert magnetic_field_msg.magnetic_field.x == pytest.approx(11.0e-6)
    assert magnetic_field_msg.magnetic_field.y == pytest.approx(12.0e-6)
    assert magnetic_field_msg.magnetic_field.z == pytest.approx(13.0e-6)

    temperature_msg = imu_harness.temperature_messages[-1]
    assert temperature_msg.header.frame_id == 'drqp/imu_link'
    assert temperature_msg.temperature == 24.5


def test_publish_measurements_skips_publish_on_sensor_failure(imu_harness):
    """Avoid publishing stale ROS messages when a sensor read fails."""
    imu_harness.fake_sensor.error = RuntimeError('i2c read failed')
    imu_harness.wait_for_subscribers()

    imu_harness.node.publish_measurements()
    imu_harness.spin_until(lambda: False, iterations=5)

    assert imu_harness.imu_messages == []
    assert imu_harness.magnetic_field_messages == []
    assert imu_harness.temperature_messages == []


def test_publish_measurements_marks_orientation_unavailable_when_none(imu_harness):
    """Publish gyro/accel even when orientation is absent; mark it unavailable."""
    imu_harness.fake_sensor.sample = ImuSample(
        orientation_wxyz=None,
        angular_velocity=(0.7, 0.8, 0.9),
        linear_acceleration=(2.1, 2.2, 2.3),
        magnetic_field_microtesla=(5.0, 6.0, 7.0),
        temperature_celsius=25.0,
    )
    imu_harness.wait_for_subscribers()

    imu_harness.node.publish_measurements()
    delivered = imu_harness.spin_until(lambda: len(imu_harness.imu_messages) > 0, iterations=20)

    assert delivered, 'Expected IMU message to be delivered without orientation'

    imu_msg = imu_harness.imu_messages[-1]
    assert imu_msg.orientation_covariance[0] == -1.0, (
        'orientation_covariance[0] must be -1 when orientation is unavailable'
    )
    assert imu_msg.angular_velocity.x == 0.7
    assert imu_msg.angular_velocity.y == 0.8
    assert imu_msg.angular_velocity.z == 0.9
    assert imu_msg.angular_velocity_covariance[0] == -1.0
    assert imu_msg.linear_acceleration.x == 2.1
    assert imu_msg.linear_acceleration.y == 2.2
    assert imu_msg.linear_acceleration.z == 2.3
    assert imu_msg.linear_acceleration_covariance[0] == -1.0


def test_constructor_wraps_sensor_construction_failures(rclpy_context):  # noqa: ARG001
    """Report backend construction failures with an actionable startup error."""
    with (
        mock.patch(
            'drqp_brain.imu_node.Bno055Sensor',
            side_effect=AttributeError('platform detection failed'),
        ),
        pytest.raises(
            SensorInitializationError,
            match='Failed to initialize the BNO055 IMU backend at I2C address 0x28',
        ) as context,
    ):
        ImuNode()

    assert isinstance(context.value.__cause__, AttributeError)


def test_main_exits_cleanly_when_sensor_backend_cannot_start():
    """Exit with code 1 and log the startup error without a traceback."""
    logger = mock.Mock()
    imu_node_constructor = mock.Mock(side_effect=SensorInitializationError('backend unavailable'))

    with (
        mock.patch('drqp_brain.imu_node.rclpy.init'),
        mock.patch('drqp_brain.imu_node.ImuNode', imu_node_constructor),
        mock.patch('drqp_brain.imu_node.rclpy.logging.get_logger', return_value=logger),
        mock.patch('drqp_brain.imu_node.rclpy.ok', return_value=True),
        mock.patch('drqp_brain.imu_node.rclpy.shutdown') as shutdown,
    ):
        with pytest.raises(SystemExit) as context:
            main()

    assert context.value.code == 1
    imu_node_constructor.assert_called_once_with()
    logger.error.assert_called_once_with('backend unavailable')
    shutdown.assert_called_once_with()
