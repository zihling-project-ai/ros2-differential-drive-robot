"""Offline IMU tests; no physical I2C device or ROS installation is needed."""

import math
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class FakeNode:
    def __init__(self, name):
        self.logger = Mock()
        self.publisher = Mock()
        self.destroyed = False

    def get_logger(self):
        return self.logger

    def create_publisher(self, *args):
        return self.publisher

    def create_timer(self, *args):
        return Mock()

    def get_clock(self):
        return Mock()

    def destroy_node(self):
        self.destroyed = True


class FakeImu:
    def __init__(self):
        self.header = SimpleNamespace(frame_id='', stamp=None)
        self.orientation_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9
        self.angular_velocity = SimpleNamespace(x=0.0, y=0.0, z=0.0)


def fake_module(name, **attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)
    return module


with patch.dict(sys.modules, {
    'rclpy': fake_module('rclpy', init=Mock(), spin=Mock(), ok=Mock(return_value=True), shutdown=Mock()),
    'rclpy.node': fake_module('rclpy.node', Node=FakeNode),
    'sensor_msgs': fake_module('sensor_msgs'),
    'sensor_msgs.msg': fake_module('sensor_msgs.msg', Imu=FakeImu),
    'smbus': fake_module('smbus', SMBus=Mock()),
}):
    from ai_car import aicar_imu


class TestImu(unittest.TestCase):
    def setUp(self):
        self.bus = Mock()
        self.registers = {0x75: 0x71, 0x6B: 0x00, 0x1B: 0x00}
        self.bus.read_byte_data.side_effect = lambda address, register: self.registers[register]
        self.bus.read_i2c_block_data.return_value = [0, 0]
        patcher = patch.object(aicar_imu.smbus, 'SMBus', return_value=self.bus)
        self.open_bus = patcher.start()
        self.addCleanup(patcher.stop)

    def test_original_configuration_and_message(self):
        node = aicar_imu.AICAR_IMU()
        self.open_bus.assert_called_once_with(1)
        self.assertEqual([tuple(call.args) for call in self.bus.write_byte_data.call_args_list],
                         [(0x68, 0x6B, 0), (0x68, 0x1B, 0)])
        self.assertEqual(aicar_imu.IMU_RATE, 200)
        self.assertEqual(node.imu_msg.header.frame_id, 'imu_link')
        self.assertEqual(node.imu_msg.orientation_covariance[0], -1.0)
        self.assertEqual(node.imu_msg.linear_acceleration_covariance[0], -1.0)

    def test_signed_16_bit_boundaries_and_scale(self):
        node = aicar_imu.AICAR_IMU()
        for data, expected in (([0, 0], 0), ([0, 1], 1), ([0x7F, 0xFF], 32767),
                               ([0x80, 0], -32768), ([0xFF, 0xFF], -1),
                               ([0xFA, 0xE2], -1310), ([5, 30], 1310)):
            with self.subTest(data=data):
                self.bus.read_i2c_block_data.return_value = data
                self.assertAlmostEqual(node.fn_read_gyro_z(), expected / 131)
        self.bus.read_i2c_block_data.assert_called_with(0x68, 0x47, 2)

    def test_valid_fast_rotation_is_not_zeroed(self):
        node = aicar_imu.AICAR_IMU()
        for raw in (26200, -26200):
            self.bus.read_i2c_block_data.return_value = list(raw.to_bytes(2, 'big', signed=True))
            node.cb_timer()
            self.assertAlmostEqual(node.imu_msg.angular_velocity.z, math.radians(raw / 131))
            self.assertIsNotNone(node.imu_msg.header.stamp)
        self.assertEqual(node.publisher.publish.call_count, 2)

    def test_read_failure_does_not_publish_stale_measurement(self):
        node = aicar_imu.AICAR_IMU()
        node.cb_timer()
        self.bus.read_i2c_block_data.side_effect = OSError('mock disconnect')
        node.cb_timer()
        self.assertEqual(node.publisher.publish.call_count, 1)
        node.logger.warning.assert_called_once()

    def test_invalid_sample_not_published(self):
        node = aicar_imu.AICAR_IMU()
        for data in ([0], [0, 256], [-1, 0], [1.2, 0], [0, 0, 0]):
            self.bus.read_i2c_block_data.return_value = data
            node.cb_timer()
        node.publisher.publish.assert_not_called()

    def test_wrong_identity_fails_before_writes(self):
        self.registers[0x75] = 0x00
        with self.assertRaises(RuntimeError):
            aicar_imu.AICAR_IMU()
        self.bus.write_byte_data.assert_not_called()
        self.bus.close.assert_called_once()

    def test_failed_configuration_readback_closes_bus(self):
        self.registers[0x1B] = 0x08
        with self.assertRaises(RuntimeError):
            aicar_imu.AICAR_IMU()
        self.bus.close.assert_called_once()

    def test_initialization_io_failure_closes_bus(self):
        self.bus.write_byte_data.side_effect = OSError('mock write failure')
        with self.assertRaises(OSError):
            aicar_imu.AICAR_IMU()
        self.bus.close.assert_called_once()

    def test_shutdown_is_idempotent(self):
        node = aicar_imu.AICAR_IMU()
        node.fn_shutdown()
        node.fn_shutdown()
        self.bus.close.assert_called_once()

    def test_main_shuts_ros_down_when_open_fails(self):
        self.open_bus.side_effect = OSError('mock open failure')
        aicar_imu.rclpy.shutdown.reset_mock()
        with self.assertRaises(OSError):
            aicar_imu.main()
        aicar_imu.rclpy.shutdown.assert_called_once()


if __name__ == '__main__':
    unittest.main()
