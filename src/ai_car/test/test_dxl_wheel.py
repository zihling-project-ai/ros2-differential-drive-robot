"""Offline wheel initialization and reset checks using a fake SDK."""

import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch


fake_sdk = ModuleType('dynamixel_sdk')
fake_sdk.COMM_SUCCESS = 0
fake_sdk.PacketHandler = Mock()
fake_sdk.PortHandler = Mock()
with patch.dict(sys.modules, {'dynamixel_sdk': fake_sdk}):
    from ai_car import dxl_wheel


class TestWheelReset(unittest.TestCase):
    def setUp(self):
        self.packet = Mock()
        self.packet.read1ByteTxRx.return_value = (dxl_wheel.VELOCITY_CONTROL_MODE, 0, 0)
        self.packet.write4ByteTxRx.return_value = (0, 0)
        self.port = object()
        handler_patch = patch.object(dxl_wheel, 'PacketHandler', return_value=self.packet)
        handler_patch.start()
        self.addCleanup(handler_patch.stop)

    def test_startup_verifies_mode_without_reconfiguring_or_enabling_torque(self):
        dxl_wheel.Wheel_Controller(self.port)
        self.assertEqual(self.packet.read1ByteTxRx.call_count, 4)
        self.packet.write1ByteTxRx.assert_not_called()
        self.assertEqual(self.packet.write4ByteTxRx.call_count, 4)
        self.assertTrue(all(
            call.args[2:] == (dxl_wheel.XM_GOAL_VELOCITY_ADDRESS, 0)
            for call in self.packet.write4ByteTxRx.call_args_list
        ))

    def test_unexpected_mode_is_reported_before_any_write(self):
        self.packet.read1ByteTxRx.side_effect = [(1, 0, 0), (3, 0, 0)]
        with self.assertRaises(RuntimeError):
            dxl_wheel.Wheel_Controller(self.port)
        self.packet.write1ByteTxRx.assert_not_called()
        self.packet.write4ByteTxRx.assert_not_called()

    def test_failed_mode_read_is_reported_before_any_write(self):
        self.packet.read1ByteTxRx.return_value = (0, -1, 0)
        with self.assertRaises(RuntimeError):
            dxl_wheel.Wheel_Controller(self.port)
        self.packet.write1ByteTxRx.assert_not_called()
        self.packet.write4ByteTxRx.assert_not_called()

    def test_reset_requests_stop_without_changing_hardware_configuration(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.reset_mock()
        wheel.reset()
        self.packet.read1ByteTxRx.assert_not_called()
        self.packet.write1ByteTxRx.assert_not_called()
        self.assertEqual(self.packet.write4ByteTxRx.call_count, 4)
        self.assertTrue(all(
            call.args[2:] == (dxl_wheel.XM_GOAL_VELOCITY_ADDRESS, 0)
            for call in self.packet.write4ByteTxRx.call_args_list
        ))

    def test_reset_attempts_all_wheels_and_reports_errors(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.write4ByteTxRx.reset_mock()
        self.packet.write4ByteTxRx.side_effect = [(-1, 0), (0, 0), (0, 1), (0, 0)]
        with self.assertRaises(RuntimeError):
            wheel.reset()
        self.assertEqual(self.packet.write4ByteTxRx.call_count, 4)
        self.packet.write1ByteTxRx.assert_not_called()


if __name__ == '__main__':
    unittest.main()
