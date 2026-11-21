"""Offline tests: the DYNAMIXEL SDK is replaced with a mock, never real hardware."""

import math
import sys
from types import ModuleType
import unittest
from unittest.mock import Mock, patch


fake_sdk = ModuleType('dynamixel_sdk')
fake_sdk.COMM_SUCCESS = 0
fake_sdk.PacketHandler = Mock()
with patch.dict(sys.modules, {'dynamixel_sdk': fake_sdk}):
    from ai_car import dxl_arm


class TestArmController(unittest.TestCase):
    def setUp(self):
        self.packet = Mock()
        self.packet.write2ByteTxRx.return_value = (0, 0)
        self.packet.write1ByteTxRx.return_value = (0, 0)
        self.port = object()
        self.handler_patch = patch.object(dxl_arm, 'PacketHandler', return_value=self.packet)
        self.sleep_patch = patch.object(dxl_arm.time, 'sleep')
        self.handler = self.handler_patch.start()
        self.sleep_patch.start()
        self.addCleanup(self.handler_patch.stop)
        self.addCleanup(self.sleep_patch.stop)

    def make_controller(self):
        return dxl_arm.Arm_Controller(self.port)

    def test_initialization_does_not_send_position_commands(self):
        self.make_controller()
        self.assertEqual(self.packet.write2ByteTxRx.call_count, 3)
        self.assertTrue(all(
            call.args[2] == dxl_arm.AX_MOVING_SPEED
            for call in self.packet.write2ByteTxRx.call_args_list
        ))
        self.packet.write1ByteTxRx.assert_not_called()

    def test_initialization_reports_communication_failure(self):
        self.packet.write2ByteTxRx.return_value = (-1, 0)
        with self.assertRaises(RuntimeError):
            self.make_controller()
        self.assertEqual(self.packet.write2ByteTxRx.call_count, 1)

    def test_invalid_joint_input_sends_no_position_commands(self):
        arm = self.make_controller()
        self.packet.write2ByteTxRx.reset_mock()
        for invalid in ([0], [0, 0, 0], [0, -1], [0, math.nan], [0, math.inf], [0, 91]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                arm.set_joint(invalid)
        self.packet.write2ByteTxRx.assert_not_called()

    def test_invalid_tool_input_sends_no_position_command(self):
        arm = self.make_controller()
        self.packet.write2ByteTxRx.reset_mock()
        for invalid in (-1, 46, math.nan, math.inf, '10'):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                arm.set_tool(invalid)
        self.packet.write2ByteTxRx.assert_not_called()

    def test_position_round_trip_on_mock_feedback(self):
        arm = self.make_controller()
        self.packet.write2ByteTxRx.reset_mock()
        arm.set_joint([45, 45])
        arm.set_tool(45)
        ticks = [call.args[3] for call in self.packet.write2ByteTxRx.call_args_list]
        self.packet.read2ByteTxRx.side_effect = [(tick, 0, 0) for tick in ticks]
        for angle in arm.get_arm():
            self.assertLess(abs(angle - math.radians(45)), dxl_arm.AX_TICK2RAD)

    def test_failed_read_is_not_returned_as_feedback(self):
        arm = self.make_controller()
        self.packet.read2ByteTxRx.side_effect = [(812, 0, 0), (0, -1, 0)]
        with self.assertRaises(RuntimeError):
            arm.get_arm()
        self.assertEqual(self.packet.read2ByteTxRx.call_count, 2)

    def test_invalid_raw_feedback_is_rejected(self):
        arm = self.make_controller()
        self.packet.read2ByteTxRx.return_value = (1024, 0, 0)
        with self.assertRaises(ValueError):
            arm.get_arm()

    def test_torque_input_is_validated(self):
        arm = self.make_controller()
        for invalid in (-1, 2, 0.5):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                arm.set_torque(invalid)
        self.packet.write1ByteTxRx.assert_not_called()

    def test_torque_off_attempts_all_motors_and_reports_failure(self):
        arm = self.make_controller()
        self.packet.write1ByteTxRx.side_effect = [(-1, 0), (0, 0), (0, 0)]
        with self.assertRaises(RuntimeError):
            arm.set_torque(0)
        self.assertEqual(self.packet.write1ByteTxRx.call_count, 3)


if __name__ == '__main__':
    unittest.main()
