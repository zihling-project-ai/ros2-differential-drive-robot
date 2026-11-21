#!/usr/bin/env python3

# Standard library
import math

# Hardware communication
import smbus

# ROS2 core and message types
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

I2C_BUS     = 1
I2C_ADDRESS = 0x68
IMU_RATE    = 200  # Hz

# MPU9250 registers
PWR_MGMT_1  = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47
WHO_AM_I    = 0x75
MPU9250_ID  = 0x71

# GYRO_CONFIG: ±250DPS → 0x00, ±500DPS → 0x08, ±1000DPS → 0x10, ±2000DPS → 0x18
GYRO_RANGE      = 0x00
GYRO_SENSITIVITY = 131.0   # LSB per deg/s at ±250DPS


class AICAR_IMU(Node):
    def __init__(self):
        super().__init__('aicar_imu')

        self.bus = None
        try:
            # 1. Connect and initialize the sensor before publishing measurements.
            self.bus = smbus.SMBus(I2C_BUS)
            self.fn_init()

            # 2. Publish sensor data independently of the odometry reset.
            self.imu_pub = self.create_publisher(Imu, '/imu', 10)

            # 3. Describe only the measurements this driver provides.
            self.imu_msg = Imu()
            self.imu_msg.header.frame_id = 'imu_link'
            self.imu_msg.orientation_covariance[0] = -1.0
            self.imu_msg.linear_acceleration_covariance[0] = -1.0
            # Only angular_velocity.z is measured; other axes are not measured.
            # Angular velocity covariance is unknown until measured on hardware.

            # 4. Schedule periodic acquisition at the original target rate.
            self.create_timer(1.0 / IMU_RATE, self.cb_timer)
        except Exception:
            self.fn_shutdown()
            self.destroy_node()
            raise

    # -- Timer callback --
    def cb_timer(self):
        # Read and convert deg/s to the ROS standard unit rad/s.
        try:
            gz = math.radians(self.fn_read_gyro_z())
        except (OSError, ValueError) as exc:
            # A failed measurement is not zero velocity. Never publish stale data.
            self.get_logger().warning(f'IMU read failed: {exc}', throttle_duration_sec=1.0)
            return

        # Keep valid rates across the full configured range, including > 2 rad/s.

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.angular_velocity.z = gz
        self.imu_pub.publish(self.imu_msg)

    # -- Functions --
    def fn_init(self):
        # Verify the expected device before writing its configuration.
        identity = self.bus.read_byte_data(I2C_ADDRESS, WHO_AM_I)
        if identity != MPU9250_ID:
            raise RuntimeError(f'Unexpected IMU device ID: 0x{identity:02X}')

        # Wake the sensor and keep the original gyroscope range.
        self.bus.write_byte_data(I2C_ADDRESS, PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(I2C_ADDRESS, GYRO_CONFIG, GYRO_RANGE)

        # Confirm the configured registers; initialization is not calibration.
        if (self.bus.read_byte_data(I2C_ADDRESS, PWR_MGMT_1) != 0x00 or
                self.bus.read_byte_data(I2C_ADDRESS, GYRO_CONFIG) != GYRO_RANGE):
            raise RuntimeError('IMU configuration readback failed')
        self.get_logger().info('IMU init OK')

    def fn_read_gyro_z(self):
        # Read both bytes together to avoid combining two different samples.
        data = self.bus.read_i2c_block_data(I2C_ADDRESS, GYRO_ZOUT_H, 2)
        if len(data) != 2 or any(not isinstance(byte, int) or not 0 <= byte <= 255 for byte in data):
            raise ValueError('Invalid gyroscope sample')
        high, low = data
        # Combine the bytes and decode the signed 16-bit two's complement value.
        val = (high << 8) | low
        if val > 32767:
            val -= 65536
        # Convert raw sensor counts to degrees per second.
        return val / GYRO_SENSITIVITY

    def fn_shutdown(self):
        # Close only our I2C resource; no hardware reset or calibration is needed.
        if self.bus is not None:
            bus, self.bus = self.bus, None
            try:
                bus.close()
            except OSError as exc:
                self.get_logger().warning(f'IMU bus close failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AICAR_IMU()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always release ROS resources, including after a failed initialization.
        try:
            if node is not None:
                try:
                    node.fn_shutdown()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
