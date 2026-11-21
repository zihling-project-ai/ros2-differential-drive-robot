"""Offline wheel driver checks using a fake SDK; no serial device is opened."""

import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch


fake_sdk = ModuleType('dynamixel_sdk')
fake_sdk.COMM_SUCCESS = 0
fake_sdk.PacketHandler = Mock()
with patch.dict(sys.modules, {'dynamixel_sdk': fake_sdk}):
    from ai_car import dxl_wheel


class TestWheelController(unittest.TestCase):
    def setUp(self):
        self.packet = Mock()
        self.packet.read1ByteTxRx.return_value = (dxl_wheel.VELOCITY_CONTROL_MODE, 0, 0)
        self.packet.write4ByteTxRx.return_value = (0, 0)
        self.packet.write1ByteTxRx.return_value = (0, 0)
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

    def test_signed_unsigned_conversion_round_trip(self):
        for value in (-0x80000000, -1, 0, 1, 0x7FFFFFFF):
            with self.subTest(value=value):
                self.assertEqual(dxl_wheel.to_int32(dxl_wheel.to_uint32(value)), value)

    def test_conversion_rejects_invalid_width_or_type(self):
        for value in (-0x80000001, 0x80000000, 1.5, True):
            with self.subTest(signed=value), self.assertRaises(ValueError):
                dxl_wheel.to_uint32(value)
        for value in (-1, 0x100000000, 1.5, True):
            with self.subTest(unsigned=value), self.assertRaises(ValueError):
                dxl_wheel.to_int32(value)

    def test_invalid_wheel_commands_do_not_write_to_any_motor(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.write4ByteTxRx.reset_mock()
        for invalid in ([1], [1, 2, 3], [1, 1.5], [True, 0], [0, 266], [-266, 0]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                wheel.set_wheel(invalid)
        self.packet.write4ByteTxRx.assert_not_called()

    def test_wheel_mapping_and_signed_encoding_are_preserved(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.write4ByteTxRx.reset_mock()
        wheel.set_wheel([12, -12])
        calls = self.packet.write4ByteTxRx.call_args_list
        self.assertEqual([call.args[1] for call in calls], list(dxl_wheel.WHEEL_IDS))
        self.assertEqual([dxl_wheel.to_int32(call.args[3]) for call in calls], [12, -12, 12, -12])

    def test_partial_velocity_write_requests_stop_on_every_wheel(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.write4ByteTxRx.reset_mock()
        self.packet.write4ByteTxRx.side_effect = [(0, 0), (-1, 0)] + [(0, 0)] * 4
        with self.assertRaises(RuntimeError):
            wheel.set_wheel([1, 1])
        stop_calls = self.packet.write4ByteTxRx.call_args_list[-4:]
        self.assertEqual([call.args[1] for call in stop_calls], list(dxl_wheel.WHEEL_IDS))
        self.assertTrue(all(call.args[3] == 0 for call in stop_calls))

    def test_get_wheel_returns_signed_ticks_in_original_order(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        self.packet.read4ByteTxRx.side_effect = [(0, 0, 0), (0xFFFFFFFF, 0, 0),
                                                   (4096, 0, 0), (0x80000000, 0, 0)]
        self.assertEqual(wheel.get_wheel(), [0, -1, 4096, -0x80000000])
        self.assertEqual(
            [call.args[1] for call in self.packet.read4ByteTxRx.call_args_list],
            list(dxl_wheel.WHEEL_IDS),
        )

    def test_invalid_or_failed_feedback_is_not_returned(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        for response, error_type in (((0, -1, 0), RuntimeError),
                                     ((0, 0, 1), RuntimeError),
                                     ((0x100000000, 0, 0), ValueError)):
            with self.subTest(response=response), self.assertRaises(error_type):
                self.packet.read4ByteTxRx.return_value = response
                wheel.get_wheel()

    def test_torque_rejects_invalid_input_and_reports_communication_error(self):
        wheel = dxl_wheel.Wheel_Controller(self.port)
        for invalid in (-1, 2, 1.5, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                wheel.set_torque(invalid)
        self.packet.write1ByteTxRx.assert_not_called()
        self.packet.write1ByteTxRx.side_effect = [(-1, 0)] + [(0, 0)] * 3
        with self.assertRaises(RuntimeError):
            wheel.set_torque(0)
        self.assertEqual(self.packet.write1ByteTxRx.call_count, 4)


if __name__ == '__main__':
    unittest.main()
