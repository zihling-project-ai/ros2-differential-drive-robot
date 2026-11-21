"""Check the real ROS 2 IMU Node and message with an entirely fake I2C bus."""

import math
import sys
from types import ModuleType
from unittest.mock import Mock

import rclpy


fake_smbus = ModuleType('smbus')
bus = Mock()
bus.read_byte_data.side_effect = lambda address, register: {0x75: 0x71, 0x6B: 0, 0x1B: 0}[register]
bus.read_i2c_block_data.return_value = [0x66, 0x58]  # 200 deg/s
fake_smbus.SMBus = Mock(return_value=bus)
sys.modules['smbus'] = fake_smbus
from ai_car import aicar_imu


def main():
    rclpy.init()
    node = None
    try:
        node = aicar_imu.AICAR_IMU()
        node.cb_timer()
        assert math.isclose(node.imu_msg.angular_velocity.z, math.radians(200))
        assert node.imu_msg.header.frame_id == 'imu_link'
        assert node.imu_msg.orientation_covariance[0] == -1.0
        bus.read_i2c_block_data.side_effect = OSError('simulated I2C failure')
        node.cb_timer()
        print('ROS 2 IMU timer, message and error logging smoke test passed (fake I2C)')
    finally:
        if node is not None:
            node.fn_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
