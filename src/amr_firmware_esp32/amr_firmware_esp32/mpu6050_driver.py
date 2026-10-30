#!/usr/bin/env python3
"""MPU6050 driver (I2C) publishing sensor_msgs/Imu on /imu/out.

Configuration used (and the scale factors that MUST match it):
  GYRO_CONFIG  = 0x00 -> +/-250 deg/s, 131 LSB/(deg/s)
  ACCEL_CONFIG = 0x00 -> +/-2 g,       16384 LSB/g
  CONFIG (DLPF)= 3    -> ~42 Hz low-pass, gyro output rate 1 kHz
  SMPLRT_DIV   = 9    -> 1 kHz / (1 + 9) = 100 Hz sample rate

The MPU6050 has no magnetometer, so there is NO absolute orientation. The message
therefore carries orientation_covariance[0] = -1 (REP-145: "orientation not provided")
and only angular velocity + linear acceleration should be fused.
"""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import smbus

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B  # start of the 14-byte burst: accel xyz, temp, gyro xyz
DEVICE_ADDRESS = 0x68

GYRO_LSB_PER_DPS = 131.0     # for GYRO_CONFIG = 0x00 (+/-250 deg/s)
ACCEL_LSB_PER_G = 16384.0    # for ACCEL_CONFIG = 0x00 (+/-2 g)
GYRO_SCALE = math.radians(1.0) / GYRO_LSB_PER_DPS   # rad/s per LSB
ACCEL_SCALE = 9.80665 / ACCEL_LSB_PER_G             # m/s^2 per LSB

# Starting-point noise estimates - tune against real data.
GYRO_VARIANCE = 1.0e-4    # (rad/s)^2
ACCEL_VARIANCE = 1.0e-2   # (m/s^2)^2


class MPU6050_Driver(Node):

    def __init__(self):
        super().__init__("mpu6050_driver")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("gyro_calibration_samples", 200)  # 0 disables calibration

        # I2C interface
        self.bus_ = None
        self.is_connected_ = False

        # Gyro bias calibration (robot must be stationary at start-up)
        self.calib_target_ = int(self.get_parameter("gyro_calibration_samples").value)
        self.calib_samples_ = []
        self.gyro_bias_ = (0.0, 0.0, 0.0)
        self.calibrated_ = self.calib_target_ <= 0

        # ROS 2 interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = self.get_parameter("frame_id").value
        self.imu_msg_.orientation.w = 1.0
        self.imu_msg_.orientation_covariance[0] = -1.0  # no orientation available
        for i in (0, 4, 8):
            self.imu_msg_.angular_velocity_covariance[i] = GYRO_VARIANCE
            self.imu_msg_.linear_acceleration_covariance[i] = ACCEL_VARIANCE

        self.init_i2c()
        self.timer_ = self.create_timer(0.01, self.timerCallback)  # 100 Hz

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
                if not self.is_connected_:
                    return

            ax, ay, az, gx, gy, gz = self.read_sample()
        except OSError:
            self.is_connected_ = False
            self.calib_samples_.clear()
            return

        if not self.calibrated_:
            self.calibrate(gx, gy, gz)
            return

        self.imu_msg_.linear_acceleration.x = ax * ACCEL_SCALE
        self.imu_msg_.linear_acceleration.y = ay * ACCEL_SCALE
        self.imu_msg_.linear_acceleration.z = az * ACCEL_SCALE
        self.imu_msg_.angular_velocity.x = (gx - self.gyro_bias_[0]) * GYRO_SCALE
        self.imu_msg_.angular_velocity.y = (gy - self.gyro_bias_[1]) * GYRO_SCALE
        self.imu_msg_.angular_velocity.z = (gz - self.gyro_bias_[2]) * GYRO_SCALE

        self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub_.publish(self.imu_msg_)

    def calibrate(self, gx, gy, gz):
        if not self.calib_samples_:
            self.get_logger().info(
                f"Calibrating gyro bias from {self.calib_target_} samples - keep the robot still")
        self.calib_samples_.append((gx, gy, gz))
        if len(self.calib_samples_) >= self.calib_target_:
            n = float(len(self.calib_samples_))
            self.gyro_bias_ = tuple(sum(s[i] for s in self.calib_samples_) / n for i in range(3))
            self.calib_samples_.clear()
            self.calibrated_ = True
            self.get_logger().info(
                "Gyro bias (rad/s): "
                + ", ".join(f"{b * GYRO_SCALE:+.5f}" for b in self.gyro_bias_))

    def init_i2c(self):
        try:
            if self.bus_ is None:
                self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)      # wake, PLL (X gyro) clock
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 9)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 3)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 0x00)
            self.bus_.write_byte_data(DEVICE_ADDRESS, ACCEL_CONFIG, 0x00)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
        except OSError:
            self.is_connected_ = False

    def read_sample(self):
        # One 14-byte burst instead of 12 separate transactions: a consistent
        # snapshot, and ~10x less I2C traffic.
        data = self.bus_.read_i2c_block_data(DEVICE_ADDRESS, ACCEL_XOUT_H, 14)
        ax, ay, az, _temp, gx, gy, gz = struct.unpack(">hhhhhhh", bytes(data))
        return ax, ay, az, gx, gy, gz


def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    rclpy.spin(mpu6050_driver)
    mpu6050_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
