#!/usr/bin/env python3
#
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

import argparse
from dataclasses import dataclass
from typing import Callable, Protocol

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.utilities
from sensor_msgs.msg import Imu, MagneticField, Temperature


@dataclass(frozen=True)
class ImuSample:
    """A normalized IMU sample collected from a hardware backend."""

    orientation_wxyz: tuple[float, float, float, float] | None
    angular_velocity: tuple[float, float, float] | None
    linear_acceleration: tuple[float, float, float] | None
    magnetic_field_microtesla: tuple[float, float, float] | None = None
    temperature_celsius: float | None = None


class ImuSensor(Protocol):
    """Protocol implemented by concrete IMU backends."""

    def read_sample(self) -> ImuSample:
        """Read and normalize the next IMU sample."""


def _as_vector3(value: object) -> tuple[float, float, float] | None:
    """Convert a 3-axis reading to a tuple of floats when available."""
    if value is None:
        return None
    vector = tuple(float(component) for component in value)
    if len(vector) != 3:
        raise ValueError(f'Expected 3 components, got {len(vector)}')
    return vector


def _as_quaternion(value: object) -> tuple[float, float, float, float] | None:
    """Convert a quaternion reading to a tuple of floats when available."""
    if value is None:
        return None
    quaternion = tuple(float(component) for component in value)
    if len(quaternion) != 4:
        raise ValueError(f'Expected 4 components, got {len(quaternion)}')
    return quaternion


class Bno055Sensor:
    """Read IMU data from a BNO055 connected over Raspberry Pi I2C."""

    def __init__(self, address: int):
        import adafruit_bno055
        import board
        import busio

        i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_bno055.BNO055_I2C(i2c, address=address)

    def read_sample(self) -> ImuSample:
        """Read the latest BNO055 sample."""
        # API reference: https://docs.circuitpython.org/projects/bno055/en/stable/api.html
        return ImuSample(
            orientation_wxyz=_as_quaternion(self._sensor.quaternion),
            angular_velocity=_as_vector3(self._sensor.gyro),
            linear_acceleration=_as_vector3(self._sensor.acceleration),
            magnetic_field_microtesla=_as_vector3(self._sensor.magnetic),
            temperature_celsius=(
                None if self._sensor.temperature is None else float(self._sensor.temperature)
            ),
        )


class ImuNode(Node):
    """Publish BNO055 IMU readings using standard ROS sensor messages."""

    def __init__(
        self,
        sensor_factory: Callable[[int], ImuSensor] | None = None,
    ):
        super().__init__('drqp_imu')

        self.declare_parameter('frame_id', 'dr_qp/imu_link')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('i2c_address', 0x28)
        self.declare_parameter('publish_temperature', True)

        publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        if publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_temperature = (
            self.get_parameter('publish_temperature').get_parameter_value().bool_value
        )
        address = self.get_parameter('i2c_address').get_parameter_value().integer_value

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.magnetic_field_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.temperature_pub = self.create_publisher(Temperature, '/imu/temperature', 10)

        if sensor_factory is None:
            sensor_factory = Bno055Sensor
        self.sensor = sensor_factory(int(address))
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_measurements)

    def publish_measurements(self):
        """Read the sensor once and publish ROS messages for the sample."""
        try:
            sample = self.sensor.read_sample()
        except Exception as exc:
            self.get_logger().warning(f'Failed to read IMU sample: {exc}')
            return

        if sample.angular_velocity is None or sample.linear_acceleration is None:
            self.get_logger().warning(
                'Skipping IMU publish because accelerometer or gyroscope data is unavailable'
            )
            return

        stamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = self.frame_id

        if sample.orientation_wxyz is None:
            imu_msg.orientation_covariance[0] = -1.0
        else:
            orientation_w, orientation_x, orientation_y, orientation_z = sample.orientation_wxyz
            imu_msg.orientation.w = orientation_w
            imu_msg.orientation.x = orientation_x
            imu_msg.orientation.y = orientation_y
            imu_msg.orientation.z = orientation_z

        angular_velocity_x, angular_velocity_y, angular_velocity_z = sample.angular_velocity
        imu_msg.angular_velocity.x = angular_velocity_x
        imu_msg.angular_velocity.y = angular_velocity_y
        imu_msg.angular_velocity.z = angular_velocity_z
        imu_msg.angular_velocity_covariance[0] = -1.0  # covariance unknown

        linear_acceleration_x, linear_acceleration_y, linear_acceleration_z = (
            sample.linear_acceleration
        )
        imu_msg.linear_acceleration.x = linear_acceleration_x
        imu_msg.linear_acceleration.y = linear_acceleration_y
        imu_msg.linear_acceleration.z = linear_acceleration_z
        imu_msg.linear_acceleration_covariance[0] = -1.0  # covariance unknown

        self.imu_pub.publish(imu_msg)

        if sample.magnetic_field_microtesla is not None:
            magnetic_field_msg = MagneticField()
            magnetic_field_msg.header.stamp = stamp
            magnetic_field_msg.header.frame_id = self.frame_id
            mag_x, mag_y, mag_z = sample.magnetic_field_microtesla
            magnetic_field_msg.magnetic_field.x = mag_x * 1e-6
            magnetic_field_msg.magnetic_field.y = mag_y * 1e-6
            magnetic_field_msg.magnetic_field.z = mag_z * 1e-6
            self.magnetic_field_pub.publish(magnetic_field_msg)

        if self.publish_temperature and sample.temperature_celsius is not None:
            temperature_msg = Temperature()
            temperature_msg.header.stamp = stamp
            temperature_msg.header.frame_id = self.frame_id
            temperature_msg.temperature = sample.temperature_celsius
            self.temperature_pub.publish(temperature_msg)


def main():
    """Entry point for the Dr.QP IMU node."""
    node = None
    try:
        parser = argparse.ArgumentParser('Dr.QP BNO055 IMU ROS node')
        filtered_args = rclpy.utilities.remove_ros_args()
        args = parser.parse_args(args=filtered_args[1:])
        rclpy.init()
        node = ImuNode(**vars(args))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # codeql[py/empty-except]
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
